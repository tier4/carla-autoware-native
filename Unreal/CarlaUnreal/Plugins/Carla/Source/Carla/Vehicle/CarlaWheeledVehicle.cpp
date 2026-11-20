// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/disable-ue4-macros.h>
#include <carla/ros2/AutowareSteeringCompensation.h>
#include <util/enable-ue4-macros.h>

#include "Carla.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaHUD.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Lights/CarlaLight.h"
#include "BlueprintLibary/LightDefaultsJsonUtils.h"
#include "Carla/Trigger/FrictionTrigger.h"
#include "Carla/Util/ActorAttacher.h"
#include "Carla/Util/EmptyActor.h"
#include "Carla/Util/BoundingBoxCalculator.h"
#include "Carla/Vegetation/VegetationManager.h"
#include "Carla/Vehicle/CarlaVehicleAnimationInstance.h"

#include <util/ue-header-guard-begin.h>
#include "Components/BoxComponent.h"
#include "Components/LightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "UObject/UnrealType.h"
#include "MovementComponents/DefaultMovementComponent.h"
#include "Rendering/SkeletalMeshRenderData.h"
#include "VehicleAnimationInstance.h"
#include "UObject/UObjectGlobals.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "PhysicsEngine/PhysicsObjectExternalInterface.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "SteeringSystem.h"
#include <util/ue-header-guard-end.h>

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

// SnapshotPose fills LocalTransforms with parent-relative bone transforms, so
// scaling those by the component transform alone never reaches world space.
static void MakePoseWorldSpace(USkeletalMeshComponent *Mesh, FPoseSnapshot &Pose)
{
  if (Mesh == nullptr)
  {
    return;
  }
  const FTransform ComponentToWorld = Mesh->GetComponentTransform();
  const int32 NumBones = FMath::Min(Pose.LocalTransforms.Num(), Mesh->GetNumBones());
  for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex)
  {
    Pose.LocalTransforms[BoneIndex] = Mesh->GetBoneTransform(BoneIndex, ComponentToWorld);
  }
}

void ACarlaWheeledVehicle::PrintROS2Message(const char* Message)
{
  FString ROSMessage = Message;
  UE_LOG(LogCarla, Warning, TEXT("ROS2 Message received: %s"), *ROSMessage);
}

ACarlaWheeledVehicle::ACarlaWheeledVehicle(const FObjectInitializer& ObjectInitializer) :
  Super(ObjectInitializer)
{
  VehicleBounds = CreateDefaultSubobject<UBoxComponent>(TEXT("VehicleBounds"));
  VehicleBounds->SetupAttachment(RootComponent);
  VehicleBounds->SetHiddenInGame(true);
  VehicleBounds->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);


  VelocityControl = CreateDefaultSubobject<UVehicleVelocityControl>(TEXT("VelocityControl"));
  VelocityControl->Deactivate();

  AccelerationControl = CreateDefaultSubobject<UVehicleAccelerationControl>(TEXT("AccelerationControl"));
  AccelerationControl->Deactivate();

  GetChaosWheeledVehicleMovementComponent()->bReverseAsBrake = false;
  BaseMovementComponent = CreateDefaultSubobject<UBaseCarlaMovementComponent>(TEXT("BaseMovementComponent"));

}

ACarlaWheeledVehicle::~ACarlaWheeledVehicle() {}

void ACarlaWheeledVehicle::BeginPlay()
{
  Super::BeginPlay();

  ActivateVehicleLightComponents();

  UDefaultMovementComponent::CreateDefaultMovementComponent(this);

  // Get constraint components and their initial transforms
  FTransform ActorInverseTransform = GetActorTransform().Inverse();
  ConstraintsComponents.Empty();
  DoorComponentsTransform.Empty();
  ConstraintDoor.Empty();
  for (FName& ComponentName : ConstraintComponentNames)
  {
    UPhysicsConstraintComponent* ConstraintComponent =
      Cast<UPhysicsConstraintComponent>(GetDefaultSubobjectByName(ComponentName));
    if (ConstraintComponent)
    {
      UPrimitiveComponent* DoorComponent = Cast<UPrimitiveComponent>(
        GetDefaultSubobjectByName(ConstraintComponent->ComponentName1.ComponentName));
      if (DoorComponent)
      {
        UE_LOG(LogCarla, Log, TEXT("Door name: %s"), *(DoorComponent->GetName()));
        FTransform ComponentWorldTransform = DoorComponent->GetComponentTransform();
        FTransform RelativeTransform = ComponentWorldTransform * ActorInverseTransform;
        DoorComponentsTransform.Add(DoorComponent, RelativeTransform);
        ConstraintDoor.Add(ConstraintComponent, DoorComponent);
        ConstraintsComponents.Add(ConstraintComponent);
        ConstraintComponent->TermComponentConstraint();
      }
      else
      {
        UE_LOG(LogCarla, Error, TEXT("Missing component for constraint: %s"), *(ConstraintComponent->GetName()));
      }
    }
  }
  // get collision disable constraints (used to prevent doors from colliding with each other)
  CollisionDisableConstraints.Empty();
  TArray<UPhysicsConstraintComponent*> Constraints;
  GetComponents(Constraints);
  for (UPhysicsConstraintComponent* Constraint : Constraints)
  {
    if (!ConstraintsComponents.Contains(Constraint))
    {
      // Closed doors are welded to the chassis. A joint between two such
      // components would constrain a Chaos particle to itself.
      Constraint->TermComponentConstraint();
      UPrimitiveComponent* CollisionDisabledComponent1 = Cast<UPrimitiveComponent>(
        GetDefaultSubobjectByName(Constraint->ComponentName1.ComponentName));
      UPrimitiveComponent* CollisionDisabledComponent2 = Cast<UPrimitiveComponent>(
        GetDefaultSubobjectByName(Constraint->ComponentName2.ComponentName));
      if (CollisionDisabledComponent1)
      {
        CollisionDisableConstraints.Add(CollisionDisabledComponent1, Constraint);
      }
      if (CollisionDisabledComponent2)
      {
        CollisionDisableConstraints.Add(CollisionDisabledComponent2, Constraint);
      }
    }
  }
  ResetConstraints();

  UChaosWheeledVehicleMovementComponent* MovementComponent = GetChaosWheeledVehicleMovementComponent();

  if (MovementComponent)
  {
    check(MovementComponent != nullptr);

    // UE5 Chaos defaults the steering input to SquaredFunction, an
    // analog-gamepad feel curve that maps a steer command s to ~s^2 of full
    // lock. CARLA clients (Traffic Manager, agents, manual control) expect
    // the classic linear mapping from command to wheel angle; under the
    // squared curve a 0.1 command yields ~1% of full lock and controllers
    // tuned for a linear response cannot corner.
    MovementComponent->SteeringInputRate.InputCurveFunction = EInputFunctionType::LinearFunction;

    // Set the friction scale to Wheel CDO and update wheel setups
    TArray<FChaosWheelSetup> NewWheelSetups = MovementComponent->WheelSetups;
    for (const FChaosWheelSetup& WheelSetup : NewWheelSetups)
    {
      UChaosVehicleWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();
      check(Wheel != nullptr);
    }

    MovementComponent->WheelSetups = NewWheelSetups;

    LastPhysicsControl = GetVehiclePhysicsControl();

    // Update physics in the Ackermann Controller
    AckermannController.UpdateVehiclePhysics(this);
  }
  AddReferenceToManager();
}

// Head beams need far more light than the other vehicle groups: under this
// project's fixed day-calibrated exposure, a visible pool on the road at
// 10-20 m takes millions of lumens (street lamps run tens of millions), and
// the beams' authored group intensity is ~30. Blinkers/brakes/fog/plate are
// close-range signal lights whose authored values read fine and must NOT get
// this factor (a multi-million-lumen blinker would strobe the whole street).
static TAutoConsoleVariable<float> CVarCarlaVehicleBeamIntensityScale(
    TEXT("carla.Light.VehicleBeamIntensityScale"),
    100000.0f,
    TEXT("Multiplier converting the authored UE4-era intensity of vehicle low/high beam ")
    TEXT("light components (and the blueprint's 'Beam Lights' group variable) to UE5 ")
    TEXT("photometric units. Set 1 to leave them untouched."),
    ECVF_Default);

void ACarlaWheeledVehicle::ActivateVehicleLightComponents()
{
  // Vehicle light components ship bAutoActivate=false; UE4 rendered inactive
  // lights anyway, UE 5.8 culls them outright, so every beam was invisible
  // regardless of intensity. Activate only -- unlike street lamps, the
  // authored automotive IES profiles (Vee/DefinedSpot beam shapes), shadow
  // flags and attenuation radii are correct as-is and must not go through
  // UCarlaLight::ActivateAndConfigureLightComponents (a 70 m radius floor on
  // a blinker or license-plate light would be absurd).
  //
  // The base vehicle blueprint assigns each light component's intensity from
  // per-group variables in its CONSTRUCTION script (so that has already
  // happened by BeginPlay) and afterwards only toggles component VISIBILITY
  // per light-state flag -- so converting the beam components right here
  // sticks. The blueprint's "Beam Lights" group variable is scaled by the
  // same factor so any later blueprint re-push writes the same converted
  // value instead of reverting the beams to authored UE4 units.
  const float BeamScale = CVarCarlaVehicleBeamIntensityScale.GetValueOnGameThread();
  TArray<ULightComponent*> LightComponents;
  GetComponents<ULightComponent>(LightComponents);
  for (ULightComponent* LightComponent : LightComponents)
  {
    if (!LightComponent->IsActive())
    {
      LightComponent->SetActive(true);
    }
    const FString Name = LightComponent->GetName();
    if (BeamScale != 1.0f &&
        (Name.Contains(TEXT("low_beam")) || Name.Contains(TEXT("high_beam"))) &&
        LightComponent->Intensity > 0.0f && LightComponent->Intensity < 1000.0f)
    {
      LightComponent->SetIntensity(LightComponent->Intensity * BeamScale);
    }
  }

  if (BeamScale != 1.0f)
  {
    for (TFieldIterator<FFloatProperty> PropIt(GetClass()); PropIt; ++PropIt)
    {
      FFloatProperty* Property = *PropIt;
      if (Property->GetAuthoredName() != TEXT("Beam Lights"))
      {
        continue;
      }
      float* ValuePtr = Property->ContainerPtrToValuePtr<float>(this);
      if (ValuePtr != nullptr && *ValuePtr > 0.0f && *ValuePtr < 1000.0f)
      {
        *ValuePtr *= BeamScale;
        UE_LOG(LogCarla, Verbose, TEXT("%s: scaled blueprint 'Beam Lights' to %f"),
            *GetName(), *ValuePtr);
      }
      break;
    }
  }

  // Light Defaults tool integration: everything above is this project's
  // pre-existing native pipeline (per-vehicle authored intensities from the
  // Blueprint's own Construction Script, scaled once here) -- it never
  // touched Config/Lights/Defaults.json, so nothing edited/saved in the
  // Vehicles tab ever reached an actual running simulation (manual_control.py
  // et al), only the editor-only Preview spawn.
  //
  // Load the saved default once here (cheap, cached for the vehicle's
  // lifetime) and apply it for the CURRENT light state -- at this point that
  // is InputControl.LightState's default (everything off), so this must
  // zero every group's material emissive, not light everything up. Do NOT
  // use ApplyVehicleLightsToSingleActor/ApplyVehicleLightsLive here: those
  // push Intensity uniformly to every exposed group at once, which is
  // correct for the editor Preview (meant to show everything lit for
  // editing) but wrong for gameplay, where a group must only glow when it is
  // actually ON.
  {
    const FString ClassName = GetClass()->GetName();
    bHasSavedVehicleLightDefault = ULightDefaultsJsonUtils::LoadVehicleLightDefault(
        ClassName, SavedVehicleLightIntensity, SavedVehicleLightGroupIntensity);
  }
  ApplyVehicleLightDefaultsForCurrentState();
}

void ACarlaWheeledVehicle::ApplyVehicleLightDefaultsForCurrentState()
{
  if (!bHasSavedVehicleLightDefault)
    return;

  const FVehicleLightState& LightState = InputControl.LightState;
  TMap<FString, bool> GroupOn;
  GroupOn.Add(TEXT("Position"), LightState.Position);
  GroupOn.Add(TEXT("Low Beam"), LightState.LowBeam);
  GroupOn.Add(TEXT("High Beam"), LightState.HighBeam);
  GroupOn.Add(TEXT("Fog"), LightState.Fog);
  GroupOn.Add(TEXT("Brake"), LightState.Brake);
  GroupOn.Add(TEXT("Reverse"), LightState.Reverse);
  GroupOn.Add(TEXT("Left Blinker"), LightState.LeftBlinker);
  GroupOn.Add(TEXT("Right Blinker"), LightState.RightBlinker);
  GroupOn.Add(TEXT("Special1"), LightState.Special1);
  GroupOn.Add(TEXT("Interior"), LightState.Interior);

  TMap<FString, float> MaterialGroupValue;
  for (const TPair<FString, bool>& Pair : GroupOn)
  {
    MaterialGroupValue.Add(Pair.Key, Pair.Value ? SavedVehicleLightIntensity : 0.0f);
  }

  ULightDefaultsJsonUtils::ApplyVehicleLightsRuntimeState(
      this, MaterialGroupValue, SavedVehicleLightGroupIntensity);

  // Authored modular lamps can opt into state control without relying on a
  // template Blueprint's fixed component list. Untagged lights retain their
  // existing Blueprint behavior and each tagged lamp retains its photometry.
  TInlineComponentArray<ULightComponent*> AuthoredLights(this);
  for (ULightComponent* Light : AuthoredLights)
  {
    bool bControlled = false;
    bool bEnabled = false;
    for (const TPair<FString, bool>& Pair : GroupOn)
    {
      if (Light->ComponentHasTag(FName(*(FString(TEXT("Carla.Light.")) + Pair.Key))))
      {
        bControlled = true;
        bEnabled |= Pair.Value;
      }
    }
    if (bControlled)
      Light->SetVisibility(bEnabled);
  }
}

void ACarlaWheeledVehicle::ResolveRiderComponentsIfNeeded()
{
  if (bRiderComponentsResolved)
  {
    return;
  }
  bRiderComponentsResolved = true;

  if (!IsTwoWheeledVehicle())
  {
    return;
  }

  RiderMeshComponent = Cast<USkeletalMeshComponent>(GetDefaultSubobjectByName(RiderMeshComponentName));
  VehicleMeshForRiderSeat = GetMesh();

  if ((RiderMeshComponent == nullptr) || (VehicleMeshForRiderSeat == nullptr) ||
      !VehicleMeshForRiderSeat->DoesSocketExist(VehicleMeshSeatSocketName))
  {
    UE_LOG(LogCarla, Warning,
        TEXT("%s: IsTwoWheeledVehicle is true but rider component '%s' or seat socket '%s' could not be resolved -- rider seat-lock disabled."),
        *GetName(), *RiderMeshComponentName.ToString(), *VehicleMeshSeatSocketName.ToString());
    RiderMeshComponent = nullptr;
    return;
  }

  // RiderMeshComponent ships parented (attached) to the vehicle mesh in
  // the Blueprint's own component hierarchy -- the engine's normal
  // attachment propagation tries to keep it glued to the parent's
  // transform on its own, continuously, independent of and in addition to
  // our own SetWorldTransform below. That's a second system moving the
  // same component: our smoothed value can only "win" until the next
  // native attachment update runs (driven by the parent's raw,
  // unsmoothed motion), which is invisible to any log of what WE compute
  // since it happens outside our own code entirely. Detaching once here
  // (KeepWorldTransform: no visual pop) makes our own positioning the
  // ONLY thing moving this component from this point on.
  RiderMeshComponent->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);

  // Doing the seat-lock from TickActor (tried first) still drifted while
  // moving: the rider's own animation gets evaluated in a later phase of
  // the frame than Actor tick, so whatever the Blueprint's per-tick logic
  // (or the animation itself) does to the rider after TickActor returns
  // still lands after our correction. OnBoneTransformsFinalized fires
  // once this component's animation/bone evaluation is fully done for
  // the frame -- there's nothing left afterwards to undo it before render.
  //
  // Neither this, a real engine attachment, nor forcing the rider to tick
  // in TG_PostPhysics (all tried in this same investigation) fixed the
  // drift -- the last one measurably made it worse, which rules out tick
  // order as the cause (forcing later ticking should help or be neutral,
  // not hurt). Async Physics is confirmed disabled project-wide too, so
  // it isn't a render-interpolation gap either. The Riding/Stopped state
  // machine's transition rules are also ruled out: Vehicle Speed <=/>
  // Stop Speed with no dead zone, but Stop Speed is a near-zero threshold
  // -- at the highway speeds where the drift is worst this never flips,
  // so the state machine isn't blending anything there. Keeping this
  // version (proven at least not to make things worse) as the baseline
  // while OnRiderBoneTransformsFinalized below logs both sides of the
  // seat-lock every frame to actually measure where the divergence comes
  // from, instead of guessing at another candidate.
  RiderMeshComponent->RegisterOnBoneTransformsFinalizedDelegate(
      FOnBoneTransformsFinalizedMultiCast::FDelegate::CreateUObject(
          this, &ACarlaWheeledVehicle::OnRiderBoneTransformsFinalized));
}

void ACarlaWheeledVehicle::OnRiderBoneTransformsFinalized()
{
  if ((RiderMeshComponent == nullptr) || (VehicleMeshForRiderSeat == nullptr))
  {
    return;
  }
  // Logging both sides of the seat-lock every frame (temporary, since
  // removed) proved the actual cause: the rider's own Anim Blueprint reads
  // vehicle-mesh sockets (Seat, then confirmed the same on the
  // handler/pedal sockets driving arm/leg IK once the seat was fixed and
  // hands/feet were still popping) in ITS Event BlueprintUpdateAnimation,
  // and on frames where that races ahead of the vehicle mesh's physics
  // update for the frame, it reads exactly last frame's transform instead
  // -- an intermittent one-frame-stale read (some frames correct, some
  // not), which is what a visible pop/teleport looks like, not a
  // continuous drift. Forcing this component's own tick group later
  // (tried and reverted) didn't help because the Blueprint's read happens
  // from its own animation update dispatch, not from this component's
  // TickComponent.
  // Fix: cache every socket's transform here (this delegate reliably
  // fires late enough in the frame to always be correct, confirmed by the
  // same log) and have the Blueprint read
  // GetCachedVehicleSocketWorldTransform(SocketName) instead of querying
  // the socket itself, for every seat/handler/pedal target in its Data
  // Gather. That traded the intermittent race for a one-frame lag that's
  // constant instead of flickering -- GetCachedVehicleSocketWorldTransform
  // extrapolates it forward using the velocity and timestamp cached here
  // (shared across sockets: they're all rigidly attached to the same
  // moving mesh) to cancel out the remaining visible trailing-behind-at-speed
  // too.
  CachedVehicleSocketTransforms.Reset();
  for (const FName& SocketName : VehicleMeshForRiderSeat->GetAllSocketNames())
  {
    CachedVehicleSocketTransforms.Add(SocketName, VehicleMeshForRiderSeat->GetSocketTransform(SocketName));
  }
  // LeftPedalGeo/RightPedalGeo are bones, not sockets (not covered by
  // GetAllSocketNames above) -- GetSocketTransform falls back to a bone
  // lookup when given a bone name, so this still works. Needed as pivots
  // for the pedal-socket special case in GetCachedVehicleSocketWorldTransform().
  CachedVehicleSocketTransforms.Add(TEXT("LeftPedalGeo"), VehicleMeshForRiderSeat->GetSocketTransform(TEXT("LeftPedalGeo")));
  CachedVehicleSocketTransforms.Add(TEXT("RightPedalGeo"), VehicleMeshForRiderSeat->GetSocketTransform(TEXT("RightPedalGeo")));
  CachedVehicleMeshTransform = VehicleMeshForRiderSeat->GetComponentTransform();
  CachedVehicleWorldVelocity = VehicleMeshForRiderSeat->GetComponentVelocity();
  CachedVehicleWorldAngularVelocityDegrees = bHasMirroredMotion ?
      FMath::RadiansToDegrees(CarlaGetActorAngularVelocityInRadians(*this)) :
      VehicleMeshForRiderSeat->GetPhysicsAngularVelocityInDegrees();
  if (const UWorld* World = GetWorld())
  {
    CachedVehicleTransformsTimeSeconds = World->GetTimeSeconds();
  }

  // See RiderSeatSmoothingSpeed's comment -- Chaos suspension has small,
  // real, high-frequency vertical jitter on the mesh every frame; this
  // low-pass filters CachedVehicleMeshTransform (used as the extrapolation
  // base in GetCachedVehicleSocketWorldTransform()) so the rider doesn't
  // inherit it 1:1. First frame snaps straight to the real transform --
  // interpolating FROM a default-constructed FTransform (identity, at the
  // world origin) would sweep the rider there for one frame.
  //
  // Only the vertical (Z) component of TRANSLATION is filtered -- X/Y
  // (forward/lateral motion) are copied through exactly, every frame,
  // with zero lag. Smoothing all of it (tried first) reintroduced a rider
  // slide: FMath::VInterpTo always trails a moving target, and that lag
  // grows with how fast the target (the vehicle) is moving -- at real
  // driving speed it was very visibly behind, indistinguishable from the
  // original slide bug. Suspension bounce is a small, purely vertical
  // wobble, so filtering only Z bounds the worst-case lag to that
  // wobble's own (small) amplitude, independent of vehicle speed.
  //
  // Smoothing Z alone didn't kill the visible bounce (confirmed: lowering
  // RiderSeatSmoothingSpeed changed nothing about it once settled, only
  // the initial snap-in speed) -- the real source is ROTATION jitter
  // (small Pitch/Roll noise from the same suspension), never smoothed
  // above for the same reason X/Y aren't: Yaw must track the handlebars
  // instantly or turning gets laggy. But every socket (Seat, handlers,
  // pedals) sits some distance from the mesh's own origin, and rotating
  // that lever arm by a jittery Pitch/Roll every frame moves its WORLD
  // position by an amount that grows with the lever arm -- invisible in
  // pure translation-Z terms but very visible as the whole rider
  // shuddering together, uniformly, exactly what was reported. Fix:
  // filter Pitch and Roll the same way as Z, but decompose through
  // Euler/FRotator instead of a straight FQuat::Slerp specifically to
  // keep Yaw untouched (Slerp-ing the whole quaternion would smooth Yaw
  // too and reintroduce steering lag, the same mistake already made once
  // with translation).
  if (!bSmoothedVehicleMeshTransformInitialized)
  {
    SmoothedVehicleMeshTransform = CachedVehicleMeshTransform;
    bSmoothedVehicleMeshTransformInitialized = true;
  }
  else if (const UWorld* World = GetWorld())
  {
    const float FrameDeltaTime = World->GetDeltaSeconds();

    FVector SmoothedTranslation = CachedVehicleMeshTransform.GetTranslation();
    SmoothedTranslation.Z = FMath::FInterpTo(
        SmoothedVehicleMeshTransform.GetTranslation().Z, SmoothedTranslation.Z,
        FrameDeltaTime, RiderSeatSmoothingSpeed);
    SmoothedVehicleMeshTransform.SetTranslation(SmoothedTranslation);

    const FRotator PreviousSmoothedRotator = SmoothedVehicleMeshTransform.Rotator();
    const FRotator RawRotator = CachedVehicleMeshTransform.Rotator();
    const FRotator NewSmoothedRotator(
        FMath::FInterpTo(PreviousSmoothedRotator.Pitch, RawRotator.Pitch, FrameDeltaTime, RiderSeatSmoothingSpeed),
        RawRotator.Yaw,
        FMath::FInterpTo(PreviousSmoothedRotator.Roll, RawRotator.Roll, FrameDeltaTime, RiderSeatSmoothingSpeed));
    SmoothedVehicleMeshTransform.SetRotation(NewSmoothedRotator.Quaternion());
  }

  // Reads back through GetCachedVehicleSocketWorldTransform() (the
  // now-smoothed pipeline above) instead of querying the seat socket
  // directly -- this hard-snaps the rider's whole root every frame, so an
  // unsmoothed read here would still show the suspension jitter on the
  // rider's hips/torso regardless of how smooth the AnimGraph's own IK
  // reads are.
  const FTransform SeatWorldTransform = GetCachedVehicleSocketWorldTransform(VehicleMeshSeatSocketName);
  const FTransform TargetTransform = FTransform(RiderSeatRelativeRotationOffset) * SeatWorldTransform;
  RiderMeshComponent->SetWorldTransform(TargetTransform);
}

void ACarlaWheeledVehicle::TickActor(float DeltaTime, enum ELevelTick TickType, FActorTickFunction& ThisTickFunction){
  Super::TickActor(DeltaTime, TickType, ThisTickFunction);

  // Calipers follow the wheel centre and steering, but never wheel spin.
  // Modular authoring opts in; existing vehicle animation is unchanged.
  if (ActorHasTag(TEXT("Carla.ModularWheelCalipers")))
  {
    UChaosWheeledVehicleMovementComponent* Movement = GetChaosWheeledVehicleMovementComponent();
    if (Movement && Movement->PhysicsVehicleOutput())
    {
      static const FName CaliperTags[] = {TEXT("Carla.Caliper.0"), TEXT("Carla.Caliper.1"),
          TEXT("Carla.Caliper.2"), TEXT("Carla.Caliper.3")};
      TInlineComponentArray<UStaticMeshComponent*> Components(this);
      for (UStaticMeshComponent* Component : Components)
      {
        for (int32 Index = 0; Index < UE_ARRAY_COUNT(CaliperTags) && Index < Movement->WheelSetups.Num() && Index < Movement->Wheels.Num(); ++Index)
        {
          if (Component->ComponentHasTag(CaliperTags[Index]))
          {
            const FQuat Rotation = GetMesh()->GetComponentQuat() *
                FRotator(0.0f, Movement->Wheels[Index]->GetSteerAngle(), 0.0f).Quaternion();
            Component->SetWorldLocationAndRotation(
                GetMesh()->GetSocketLocation(Movement->WheelSetups[Index].BoneName), Rotation);
            break;
          }
        }
      }
    }
  }

  // Two-wheeled vehicles: resolves (once) the rider components and
  // attaches the rider to the seat socket -- see ResolveRiderComponentsIfNeeded's
  // comment for why a real attachment, not a per-frame transform copy.
  ResolveRiderComponentsIfNeeded();

  // See GetPedalRotation()'s comment: a continuously-accumulated angle
  // instead of the vehicle mesh's looping Timeline, so the pedal bone
  // (and the rider's foot IK riding on it) never pops.
  if (IsTwoWheeledVehicle())
  {
    PedalRotationAngle = FMath::Fmod(
        PedalRotationAngle + GetVehicleForwardSpeed() * PedalRotationDegreesPerCm * DeltaTime,
        360.0f);
  }

  // When velocity/acceleration control is active, flush control every frame even without AI controller
  if (VelocityControl->IsActive() || AccelerationControl->IsActive())
  {
    FlushVehicleControl(DeltaTime);
  }

  FPoseSnapshot pose;
  GetMesh()->SnapshotPose(pose);
  MakePoseWorldSpace(GetMesh(), pose);

  WorldTransformedPose = pose;

}

bool ACarlaWheeledVehicle::IsInVehicleRange(const FVector& Location) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::IsInVehicleRange);

  return FoliageBoundingBox.IsInside(Location);
}

void ACarlaWheeledVehicle::UpdateDetectionBox()
{
  const FTransform GlobalTransform = GetActorTransform();
  const FVector Vec{ DetectionSize, DetectionSize, DetectionSize };
  FBox Box = FBox(-Vec, Vec);
  const FTransform NonScaledTransform(GlobalTransform.GetRotation(), GlobalTransform.GetLocation(), { 1.0f, 1.0f, 1.0f });
  FoliageBoundingBox = Box.TransformBy(NonScaledTransform);
}

const TArray<int32> ACarlaWheeledVehicle::GetFoliageInstancesCloseToVehicle(const UInstancedStaticMeshComponent* Component) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::GetFoliageInstancesCloseToVehicle);
  return Component->GetInstancesOverlappingBox(FoliageBoundingBox);
}

FBox ACarlaWheeledVehicle::GetDetectionBox() const
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::GetDetectionBox);
  return FoliageBoundingBox;
}

float ACarlaWheeledVehicle::GetDetectionSize() const
{
  return DetectionSize;
}

void ACarlaWheeledVehicle::DrawFoliageBoundingBox() const
{
  const FVector& Center = FoliageBoundingBox.GetCenter();
  const FVector& Extent = FoliageBoundingBox.GetExtent();
  const FQuat& Rotation = GetActorQuat();
  DrawDebugBox(GetWorld(), Center, Extent, Rotation, FColor::Magenta, false, 0.0f, 0, 5.0f);
}

FBoxSphereBounds ACarlaWheeledVehicle::GetBoxSphereBounds() const
{
  ALargeMapManager* LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld());
  if (LargeMap)
  {
    FTransform GlobalTransform = LargeMap->LocalToGlobalTransform(GetActorTransform());
    return VehicleBounds->CalcBounds(GlobalTransform);
  }
  return VehicleBounds->CalcBounds(GetActorTransform());
}

void ACarlaWheeledVehicle::AdjustVehicleBounds()
{
  FBoundingBox BoundingBox = UBoundingBoxCalculator::GetVehicleBoundingBox(this);

  const FTransform& CompToWorldTransform = RootComponent->GetComponentTransform();
  const FRotator Rotation = CompToWorldTransform.GetRotation().Rotator();
  const FVector Translation = CompToWorldTransform.GetLocation();
  const FVector Scale = CompToWorldTransform.GetScale3D();

  // Invert BB origin to local space
  BoundingBox.Origin -= Translation;
  BoundingBox.Origin = Rotation.UnrotateVector(BoundingBox.Origin);
  BoundingBox.Origin /= Scale;

  // Prepare Box Collisions
  FTransform Transform;
  Transform.SetTranslation(BoundingBox.Origin);
  VehicleBounds->SetRelativeTransform(Transform);
  VehicleBounds->SetBoxExtent(BoundingBox.Extent);
}

// =============================================================================
// -- Get functions ------------------------------------------------------------
// =============================================================================

float ACarlaWheeledVehicle::GetVehicleForwardSpeed() const
{
  if (bHasMirroredMotion)
  {
    return MirroredForwardSpeed;
  }
  return BaseMovementComponent->GetVehicleForwardSpeed();
}

FVector ACarlaWheeledVehicle::GetVehicleOrientation() const
{
  return GetVehicleTransform().GetRotation().GetForwardVector();
}

int32 ACarlaWheeledVehicle::GetVehicleCurrentGear() const
{
  if (bHasMirroredGear)
  {
    return MirroredGear;
  }
  // Acceleration control (the Autoware command path) sets the forward speed
  // directly and never engages the transmission, which stays in neutral (0).
  // It only drives forward, so report the first forward gear.
  if (AccelerationControl->IsActive())
  {
    return 1;
  }
  return BaseMovementComponent->GetVehicleCurrentGear();
}

FTransform ACarlaWheeledVehicle::GetVehicleBoundingBoxTransform() const
{
  return VehicleBounds->GetRelativeTransform();
}

FVector ACarlaWheeledVehicle::GetVehicleBoundingBoxExtent() const
{
  return VehicleBounds->GetScaledBoxExtent();
}

float ACarlaWheeledVehicle::GetMaximumSteerAngle() const
{
  UChaosWheeledVehicleMovementComponent* MovementComponent = GetChaosWheeledVehicleMovementComponent();
  // The runtime wheel holds the applied physics control, the class the defaults.
  if (MovementComponent != nullptr && MovementComponent->Wheels.Num() > 0 &&
      MovementComponent->Wheels[0] != nullptr)
  {
    return MovementComponent->Wheels[0]->MaxSteerAngle;
  }
  if (MovementComponent != nullptr && MovementComponent->WheelSetups.Num() > 0 &&
      MovementComponent->WheelSetups[0].WheelClass != nullptr)
  {
    const UChaosVehicleWheel* FrontWheel =
        Cast<UChaosVehicleWheel>(MovementComponent->WheelSetups[0].WheelClass->GetDefaultObject());
    if (FrontWheel != nullptr)
    {
      return FrontWheel->MaxSteerAngle;
    }
  }
  // A vehicle whose movement component has no wheel setups (e.g. a rig whose
  // configuration did not survive the UE5 migration) must not assert the
  // whole server down from BeginPlay; report a sensible steering default.
  UE_LOG(LogCarla, Warning, TEXT("%s: no wheel setups, returning default max steer angle"), *GetName());
  return 35.0f;
}

// Segments of the steer input range [0, 1] that the tire angle table samples.
static constexpr int32 CarlaSteerTireAngleTableSegments = 40;

// Bicycle-model angle of a pair of front wheels steered to LeftRad and
// RightRad (both in [0, pi/2)): the angle whose cotangent is the mean of the
// two wheels' cotangents. atan(2 / (cot L + cot R)) rewritten with
// cot L + cot R = sin(L + R) / (sin L sin R), so that it is 0 at 0.
static float CarlaBicycleTireAngle(float LeftRad, float RightRad)
{
  const float Numerator = 2.0f * FMath::Sin(LeftRad) * FMath::Sin(RightRad);
  if (Numerator <= 0.0f)
  {
    return 0.0f;
  }
  return FMath::Atan2(Numerator, FMath::Sin(LeftRad + RightRad));
}

// Tire angle of a table built by UpdateSteerTireAngleTable (at least two
// samples) at the steer input magnitude SteerMagnitude.
static float CarlaSteerTireAngleAt(const TArray<float> &Table, float SteerMagnitude)
{
  const int32 Segments = Table.Num() - 1;
  const float X = FMath::Clamp(SteerMagnitude, 0.0f, 1.0f) * static_cast<float>(Segments);
  const int32 K = FMath::Min(static_cast<int32>(X), Segments - 1);
  return FMath::Lerp(Table[K], Table[K + 1], X - static_cast<float>(K));
}

void ACarlaWheeledVehicle::UpdateSteerTireAngleTable() const
{
  const UChaosWheeledVehicleMovementComponent *Movement = GetChaosWheeledVehicleMovementComponent();

  // Everything the table depends on that can change at run time:
  // ApplyVehiclePhysicsControl sets the wheels' maximum steer angles and
  // whether they steer, and the runtime wheels only exist once the Chaos
  // vehicle has finished its physics initialization, after the spawn frame.
  TArray<float> Key;
  if (Movement == nullptr)
  {
    Key.Add(-1.0f);
  }
  else
  {
    Key.Add(static_cast<float>(Movement->WheelSetups.Num()));
    Key.Add(static_cast<float>(Movement->Wheels.Num()));
    Key.Add(static_cast<float>(static_cast<uint8>(Movement->SteeringSetup.SteeringType)));
    Key.Add(Movement->SteeringSetup.AngleRatio);
    for (int32 i = 0; i < Movement->Wheels.Num(); ++i)
    {
      const UChaosVehicleWheel *Wheel = Movement->Wheels[i];
      Key.Add((Wheel != nullptr && Wheel->bAffectedBySteering) ? Wheel->MaxSteerAngle : -1.0f);
    }
  }
  if (Key == SteerTireAngleTableKey)
  {
    return;
  }
  SteerTireAngleTableKey = MoveTemp(Key);
  SteerTireAngleTable.Reset();
  SteerTireAngleLinearMaxRad = FMath::DegreesToRadians(GetMaximumSteerAngle());

  const int32 WheelCount = (Movement != nullptr) ? Movement->WheelSetups.Num() : 0;
  if (WheelCount == 0 || Movement->Wheels.Num() != WheelCount)
  {
    UE_LOG(LogCarla, Verbose,
        TEXT("SteerTireAngleTable vehicle=%s result=linear reason=no_chaos_wheels max_rad=%.4f"),
        *GetName(), SteerTireAngleLinearMaxRad);
    return;
  }

  // The wheels' resting positions in the vehicle frame [cm], placed as the
  // engine places them (UChaosVehicleMovementComponent::GetWheelRestingPosition,
  // which is protected): the wheel bone, plus the wheel class offset, plus the
  // wheel setup's additional offset. The wheel track dimensions follow
  // UChaosWheeledVehicleMovementComponent::CalculateWheelLayoutDimensions
  // (also protected): twice the largest |X| (wheelbase) and |Y| (track).
  const USkeletalMeshComponent *VehicleMesh = GetMesh();
  TArray<FVector> RestingPositions;
  RestingPositions.Reserve(WheelCount);
  FVector2D WheelTrackDimensions(0.0, 0.0);
  for (int32 i = 0; i < WheelCount; ++i)
  {
    const FChaosWheelSetup &WheelSetup = Movement->WheelSetups[i];
    FVector Position = WheelSetup.AdditionalOffset;
    if (const UChaosVehicleWheel *WheelDefaults = WheelSetup.WheelClass.GetDefaultObject())
    {
      Position += WheelDefaults->Offset;
    }
    if (VehicleMesh != nullptr)
    {
      Position += VehicleMesh->GetSocketTransform(WheelSetup.BoneName, RTS_Component).GetLocation();
    }
    RestingPositions.Add(Position);
    WheelTrackDimensions.X = FMath::Max(WheelTrackDimensions.X, FMath::Abs(Position.X));
    WheelTrackDimensions.Y = FMath::Max(WheelTrackDimensions.Y, FMath::Abs(Position.Y));
  }
  WheelTrackDimensions *= 2.0;

  // The front-most steered wheel on each side (resting Y < 0 is left).
  int32 LeftIndex = INDEX_NONE;
  int32 RightIndex = INDEX_NONE;
  int32 CentreIndex = INDEX_NONE;
  for (int32 i = 0; i < WheelCount; ++i)
  {
    const UChaosVehicleWheel *Wheel = Movement->Wheels[i];
    if (Wheel == nullptr || !Wheel->bAffectedBySteering || Wheel->MaxSteerAngle <= 0.0f)
    {
      continue;
    }
    const double Side = RestingPositions[i].Y;
    int32 &Index = (Side < 0.0) ? LeftIndex : ((Side > 0.0) ? RightIndex : CentreIndex);
    if (Index == INDEX_NONE || RestingPositions[i].X > RestingPositions[Index].X)
    {
      Index = i;
    }
  }
  const bool bWheelPair = (LeftIndex != INDEX_NONE) && (RightIndex != INDEX_NONE);
  const int32 SingleIndex = (CentreIndex != INDEX_NONE) ? CentreIndex
      : ((LeftIndex != INDEX_NONE) ? LeftIndex : RightIndex);
  if (!bWheelPair && SingleIndex == INDEX_NONE)
  {
    UE_LOG(LogCarla, Verbose,
        TEXT("SteerTireAngleTable vehicle=%s result=linear reason=no_steered_wheel max_rad=%.4f"),
        *GetName(), SteerTireAngleLinearMaxRad);
    return;
  }

  // The engine's steering model, built as the movement component builds it
  // for the physics thread, from a copy of the steering setup: filling the
  // physics config of the component's own setup would rewrite the config the
  // running steering simulation reads.
  FVehicleSteeringConfig SteeringConfig = Movement->SteeringSetup;
  Chaos::FSimpleSteeringSim SteeringSim(&SteeringConfig.GetPhysicsSteeringConfig(WheelTrackDimensions));

  // Wheel angle [rad] at a steer input; WheelSide is the sign of the wheel's
  // resting Y, as in the engine's ProcessSteering.
  const auto WheelAngleRad = [&](int32 WheelIndex, float Steer) -> float
  {
    const float MaxSteerAngleDeg = Movement->Wheels[WheelIndex]->MaxSteerAngle;
    const float WheelSide = static_cast<float>(RestingPositions[WheelIndex].Y);
    return FMath::Abs(FMath::DegreesToRadians(
        SteeringSim.GetSteeringAngle(Steer, MaxSteerAngleDeg, WheelSide)));
  };

  // Steering is mirror-symmetric, so the table covers |steer| in [0, 1] and
  // the getters restore the sign; zero input is zero angle.
  TArray<float> Table;
  Table.SetNumZeroed(CarlaSteerTireAngleTableSegments + 1);
  bool bIncreasing = true;
  for (int32 k = 1; k <= CarlaSteerTireAngleTableSegments; ++k)
  {
    const float Steer = static_cast<float>(k) / static_cast<float>(CarlaSteerTireAngleTableSegments);
    Table[k] = bWheelPair
        ? CarlaBicycleTireAngle(WheelAngleRad(LeftIndex, Steer), WheelAngleRad(RightIndex, Steer))
        : WheelAngleRad(SingleIndex, Steer);
    // Also false for a NaN.
    if (!(Table[k] > Table[k - 1]))
    {
      bIncreasing = false;
    }
  }
  if (!bIncreasing)
  {
    // The inverse needs a strictly increasing table.
    UE_LOG(LogCarla, Warning,
        TEXT("SteerTireAngleTable vehicle=%s result=linear reason=not_increasing steering_type=%d max_rad=%.4f"),
        *GetName(), static_cast<int32>(Movement->SteeringSetup.SteeringType), SteerTireAngleLinearMaxRad);
    return;
  }
  SteerTireAngleTable = MoveTemp(Table);

  UE_LOG(LogCarla, Verbose,
      TEXT("SteerTireAngleTable vehicle=%s result=table steering_type=%d angle_ratio=%.3f ")
      TEXT("wheelbase_cm=%.1f track_cm=%.1f left_wheel=%d right_wheel=%d single_wheel=%d ")
      TEXT("tire_rad_at_0.10=%.4f tire_rad_at_0.25=%.4f tire_rad_at_0.50=%.4f ")
      TEXT("tire_rad_at_0.70=%.4f tire_rad_at_1.00=%.4f"),
      *GetName(),
      static_cast<int32>(Movement->SteeringSetup.SteeringType),
      Movement->SteeringSetup.AngleRatio,
      WheelTrackDimensions.X, WheelTrackDimensions.Y,
      bWheelPair ? LeftIndex : INDEX_NONE,
      bWheelPair ? RightIndex : INDEX_NONE,
      bWheelPair ? INDEX_NONE : SingleIndex,
      CarlaSteerTireAngleAt(SteerTireAngleTable, 0.10f),
      CarlaSteerTireAngleAt(SteerTireAngleTable, 0.25f),
      CarlaSteerTireAngleAt(SteerTireAngleTable, 0.50f),
      CarlaSteerTireAngleAt(SteerTireAngleTable, 0.70f),
      CarlaSteerTireAngleAt(SteerTireAngleTable, 1.00f));
}

float ACarlaWheeledVehicle::GetAutowareSteerInput(float TireAngleRad) const
{
  return bUsePerVehicleAutowareSteeringMap
      ? GetSteerInputForTireAngle(TireAngleRad)
      : carla::ros2::autoware_steering_compensation::GetSteeringInput(TireAngleRad);
}

float ACarlaWheeledVehicle::GetAutowareTireAngle(float Steer) const
{
  return bUsePerVehicleAutowareSteeringMap
      ? GetTireAngleForSteerInput(Steer)
      : carla::ros2::autoware_steering_compensation::GetSteeringOutput(Steer);
}

float ACarlaWheeledVehicle::GetTireAngleForSteerInput(float Steer) const
{
  if (FMath::IsNaN(Steer))
  {
    return 0.0f;
  }
  UpdateSteerTireAngleTable();
  const float Magnitude = FMath::Min(FMath::Abs(Steer), 1.0f);
  const float Angle = (SteerTireAngleTable.Num() > 0)
      ? CarlaSteerTireAngleAt(SteerTireAngleTable, Magnitude)
      : Magnitude * SteerTireAngleLinearMaxRad;
  return (Steer < 0.0f) ? -Angle : Angle;
}

float ACarlaWheeledVehicle::GetSteerInputForTireAngle(float TireAngleRad) const
{
  if (FMath::IsNaN(TireAngleRad))
  {
    return 0.0f;
  }
  UpdateSteerTireAngleTable();
  const float Magnitude = FMath::Abs(TireAngleRad);
  const TArray<float> &Table = SteerTireAngleTable;
  float Input = 0.0f;
  if (Table.Num() > 0)
  {
    if (Magnitude >= Table.Last())
    {
      Input = 1.0f;
    }
    else
    {
      // Table[0] == 0 <= Magnitude < Table.Last() and the table increases
      // strictly, so this stops inside the table.
      int32 K = 1;
      while (Table[K] <= Magnitude)
      {
        ++K;
      }
      const float T = (Magnitude - Table[K - 1]) / (Table[K] - Table[K - 1]);
      Input = (static_cast<float>(K - 1) + T) / static_cast<float>(Table.Num() - 1);
    }
  }
  else if (SteerTireAngleLinearMaxRad > 0.0f)
  {
    Input = Magnitude / SteerTireAngleLinearMaxRad;
  }
  Input = FMath::Min(Input, 1.0f);
  return (TireAngleRad < 0.0f) ? -Input : Input;
}

// =============================================================================
// -- Set functions ------------------------------------------------------------
// =============================================================================

void ACarlaWheeledVehicle::FlushVehicleControl()
{
  FlushVehicleControl(GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f);
}

void ACarlaWheeledVehicle::FlushVehicleControl(float DeltaTime)
{
  if (bAckermannControlActive) {
    AckermannController.UpdateVehicleState(this);
    AckermannController.RunLoop(InputControl.Control);
  }

  // tier4: optional steering output shaping (first-order lag, then rate limit),
  // set with set_vehicle_steer_first_order_lag_tau / set_vehicle_steer_rate_limit.
  // Both are off by default, and then the control is applied unchanged, as upstream.
  // The shaped value only goes to the wheels: InputControl keeps the commanded steer,
  // so sticky-control clients and every other writer of InputControl (Ackermann
  // controller, ROS 2, AI controller) keep steering towards their target.
  const float CommandedSteer = InputControl.Control.Steer;
  float AppliedSteer = CommandedSteer;
  if (DeltaTime > 0.0f && SteerFirstOrderLagTauS > 0.0f)
  {
    const float Alpha = 1.0f - FMath::Exp(-DeltaTime / SteerFirstOrderLagTauS);
    AppliedSteer = LastSteerApplied + Alpha * (CommandedSteer - LastSteerApplied);
  }
  if (DeltaTime > 0.0f && SteerRateLimit1ps > 0.0f)
  {
    const float MaxDelta = SteerRateLimit1ps * DeltaTime;
    AppliedSteer = FMath::Clamp(
        AppliedSteer,
        LastSteerApplied - MaxDelta,
        LastSteerApplied + MaxDelta);
  }
  InputControl.Control.Steer = AppliedSteer;

  // A Multi-GPU mirror's idle drivetrain reports gear 0: it keeps the
  // primary's, replayed with the control.
  const int32 ReplayedGear = InputControl.Control.Gear;
  BaseMovementComponent->ProcessControl(InputControl.Control);
  if (bHasMirroredGear)
  {
    InputControl.Control.Gear = ReplayedGear;
  }
  InputControl.Control.bReverse = InputControl.Control.Gear < 0;
  LastAppliedControl = InputControl.Control;
  LastSteerApplied = AppliedSteer;
  if (AppliedSteer != CommandedSteer)
  {
    InputControl.Control.Steer = CommandedSteer;
  }
  InputControl.Priority = EVehicleInputPriority::INVALID;
}

void ACarlaWheeledVehicle::SetThrottleInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Throttle = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetSteeringInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Steer = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetBrakeInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Brake = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetReverse(const bool Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.bReverse = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetHandbrakeInput(const bool Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.bHandBrake = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

TArray<float> ACarlaWheeledVehicle::GetWheelsFrictionScale()
{
  TArray<float> WheelsFrictionScale;
  UChaosWheeledVehicleMovementComponent* Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement)
  {
    check(Movement != nullptr);

    for (auto& Wheel : Movement->Wheels)
    {
      WheelsFrictionScale.Add(Wheel->FrictionForceMultiplier);
    }
  }

  return WheelsFrictionScale;
}

void ACarlaWheeledVehicle::SetWheelsFrictionScale(TArray<float>& WheelsFrictionScale)
{
  UChaosWheeledVehicleMovementComponent* Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement)
  {
    // The runtime wheel array is empty until the Chaos vehicle finishes its
    // physics initialization: don't crash on early or mismatched requests.
    const int32 Count = FMath::Min(Movement->Wheels.Num(), WheelsFrictionScale.Num());
    for (int32 i = 0; i < Count; ++i)
    {
      Movement->SetWheelFrictionMultiplier(i, WheelsFrictionScale[i]);
      Movement->Wheels[i]->FrictionForceMultiplier = WheelsFrictionScale[i];
      if (LastPhysicsControl.Wheels.IsValidIndex(i))
      {
        LastPhysicsControl.Wheels[i].FrictionForceMultiplier = WheelsFrictionScale[i];
      }
    }
  }
}

FVehiclePhysicsControl ACarlaWheeledVehicle::GetVehiclePhysicsControl() const
{
  FVehiclePhysicsControl PhysicsControl = { };
  auto VehicleMovComponentPtr = GetChaosWheeledVehicleMovementComponent();
  check(VehicleMovComponentPtr != nullptr);
  auto& VehicleMovComponent = *VehicleMovComponentPtr;
  auto& EngineSetup = VehicleMovComponent.EngineSetup;
  auto RCurve = EngineSetup.TorqueCurve.GetRichCurve();
  check(RCurve != nullptr);
  PhysicsControl.TorqueCurve = *RCurve;
  PhysicsControl.MaxTorque = EngineSetup.MaxTorque;
  PhysicsControl.MaxRPM = EngineSetup.MaxRPM;
  PhysicsControl.IdleRPM = EngineSetup.EngineIdleRPM;
  PhysicsControl.BrakeEffect = EngineSetup.EngineBrakeEffect;
  PhysicsControl.RevUpMOI = EngineSetup.EngineRevUpMOI;
  PhysicsControl.RevDownRate = EngineSetup.EngineRevDownRate;
  auto& DifferentialSetup = VehicleMovComponent.DifferentialSetup;
  PhysicsControl.DifferentialType = (uint8_t)DifferentialSetup.DifferentialType;
  PhysicsControl.FrontRearSplit = DifferentialSetup.FrontRearSplit;
  auto& TransmissionSetup = VehicleMovComponent.TransmissionSetup;
  PhysicsControl.bUseAutomaticGears = TransmissionSetup.bUseAutomaticGears;
  PhysicsControl.GearChangeTime = TransmissionSetup.GearChangeTime;
  PhysicsControl.FinalRatio = TransmissionSetup.FinalRatio;
  PhysicsControl.ForwardGearRatios = TransmissionSetup.ForwardGearRatios;
  PhysicsControl.ReverseGearRatios = TransmissionSetup.ReverseGearRatios;
  PhysicsControl.ChangeUpRPM = TransmissionSetup.ChangeUpRPM;
  PhysicsControl.ChangeDownRPM = TransmissionSetup.ChangeDownRPM;
  PhysicsControl.TransmissionEfficiency = TransmissionSetup.TransmissionEfficiency;
  PhysicsControl.Mass = VehicleMovComponent.Mass;
  PhysicsControl.DragCoefficient = VehicleMovComponent.DragCoefficient;
  PhysicsControl.CenterOfMass = GetCenterOfMass(VehicleMovComponent);
  PhysicsControl.ChassisWidth = VehicleMovComponent.ChassisWidth;
  PhysicsControl.ChassisHeight = VehicleMovComponent.ChassisHeight;
  PhysicsControl.DownforceCoefficient = VehicleMovComponent.DownforceCoefficient;
  PhysicsControl.DragArea = VehicleMovComponent.DragArea;
  PhysicsControl.InertiaTensorScale = VehicleMovComponent.InertiaTensorScale;
  PhysicsControl.SleepThreshold = VehicleMovComponent.SleepThreshold;
  PhysicsControl.SleepSlopeLimit = VehicleMovComponent.SleepSlopeLimit;
  RCurve = VehicleMovComponent.SteeringSetup.SteeringCurve.GetRichCurve();
  check(RCurve != nullptr);
  PhysicsControl.SteeringCurve = *RCurve;
  PhysicsControl.UseSweepWheelCollision = false;
  // Wheels Setup. The runtime wheel instances (Wheels) are only created once
  // the Chaos vehicle finishes its physics initialization, which happens after
  // the spawn frame: a client asking for the physics control right after
  // spawning would index an empty array (and crash the server). Fall back to
  // each wheel class' defaults until the runtime instances exist.
  PhysicsControl.Wheels.SetNum(VehicleMovComponent.WheelSetups.Num());
  for (int32 i = 0; i < PhysicsControl.Wheels.Num(); ++i)
  {
    const UChaosVehicleWheel* InPtr = nullptr;
    if (VehicleMovComponent.Wheels.IsValidIndex(i))
    {
      InPtr = VehicleMovComponent.Wheels[i];
    }
    else if (auto WheelClass = VehicleMovComponent.WheelSetups[i].WheelClass)
    {
      InPtr = WheelClass->GetDefaultObject<UChaosVehicleWheel>();
    }
    if (InPtr == nullptr)
    {
      continue;
    }
    auto& In = *InPtr;
    auto& Out = PhysicsControl.Wheels[i];
    Out.AxleType = In.AxleType;
    Out.Offset = In.Offset;
    Out.WheelRadius = In.WheelRadius;
    Out.WheelWidth = In.WheelWidth;
    Out.WheelMass = In.WheelMass;
    Out.CorneringStiffness = In.CorneringStiffness;
    Out.FrictionForceMultiplier = In.FrictionForceMultiplier;
    Out.SideSlipModifier = In.SideSlipModifier;
    Out.SlipThreshold = In.SlipThreshold;
    Out.SkidThreshold = In.SkidThreshold;
    Out.MaxSteerAngle = In.MaxSteerAngle;
    Out.bAffectedBySteering = In.bAffectedBySteering;
    Out.bAffectedByBrake = In.bAffectedByBrake;
    Out.bAffectedByHandbrake = In.bAffectedByHandbrake;
    Out.bAffectedByEngine = In.bAffectedByEngine;
    Out.bABSEnabled = In.bABSEnabled;
    Out.bTractionControlEnabled = In.bTractionControlEnabled;
    Out.MaxWheelspinRotation = In.MaxWheelspinRotation;
    Out.ExternalTorqueCombineMethod = In.ExternalTorqueCombineMethod;
    auto SlipCurve = In.LateralSlipGraph.GetRichCurveConst();
    check(SlipCurve != nullptr);
    Out.LateralSlipGraph = *SlipCurve;
    Out.SuspensionAxis = In.SuspensionAxis;
    Out.SuspensionForceOffset = In.SuspensionForceOffset;
    Out.SuspensionMaxRaise = In.SuspensionMaxRaise;
    Out.SuspensionMaxDrop = In.SuspensionMaxDrop;
    Out.SuspensionDampingRatio = In.SuspensionDampingRatio;
    Out.WheelLoadRatio = In.WheelLoadRatio;
    Out.SpringRate = In.SpringRate;
    Out.SpringPreload = In.SpringPreload;
    Out.SuspensionSmoothing = In.SuspensionSmoothing;
    Out.RollbarScaling = In.RollbarScaling;
    Out.SweepShape = In.SweepShape;
    Out.SweepType = In.SweepType;
    Out.MaxBrakeTorque = In.MaxBrakeTorque;
    Out.MaxHandBrakeTorque = In.MaxHandBrakeTorque;
    Out.WheelIndex = In.WheelIndex;
    // UChaosVehicleWheel's Location, OldLocation and Velocity are PhysX
    // leftovers that Chaos never updates, so they are always zero.
    // The wheel centre moves with the vehicle body: its velocity is the body's at that point (vehicles do not tick, so nothing is cached).
    Out.Location = GetWheelWorldLocation(i);
    const FBodyInstance *RootBody = GetMesh()->GetBodyInstance();
    Out.Velocity = RootBody != nullptr && RootBody->IsValidBodyInstance() ?
        RootBody->GetUnrealWorldVelocityAtPoint(Out.Location) : FVector::ZeroVector;
    Out.OldLocation = Out.Location - Out.Velocity * GetWorld()->GetDeltaSeconds();
    // If some of the wheels use Spherecast, it will be used for everyone
    // if we call ApplyPhisicsControl() as it has no sense to have it only for one.
    if (In.SweepShape == ESweepShape::Spherecast)
      PhysicsControl.UseSweepWheelCollision = true;
  }
  return PhysicsControl;
}

FVector ACarlaWheeledVehicle::GetCenterOfMass(UChaosWheeledVehicleMovementComponent& VehicleMovComponent) const
{
  if (VehicleMovComponent.bEnableCenterOfMassOverride)
  {
    return VehicleMovComponent.CenterOfMassOverride;
  }
  else {
    auto PrimitiveComponentPtr = Cast<UPrimitiveComponent>(VehicleMovComponent.UpdatedComponent);
    check(PrimitiveComponentPtr != nullptr);
    auto& PrimitiveComponent = *PrimitiveComponentPtr;
    return PrimitiveComponent.BodyInstance.COMNudge;
  }
}

FVehicleLightState ACarlaWheeledVehicle::GetVehicleLightState() const
{
  return InputControl.LightState;
}

void ACarlaWheeledVehicle::RestoreVehiclePhysicsControl()
{
  ApplyVehiclePhysicsControl(LastPhysicsControl);
}

void ACarlaWheeledVehicle::ApplyVehiclePhysicsControl(
  const FVehiclePhysicsControl& PhysicsControl)
{
  LastPhysicsControl = PhysicsControl;
  auto VehicleMovComponentPtr = GetChaosWheeledVehicleMovementComponent();
  check(VehicleMovComponentPtr != nullptr);
  auto& VehicleMovComponent = *VehicleMovComponentPtr;
  auto& EngineSetup = VehicleMovComponent.EngineSetup;
  EngineSetup.TorqueCurve.EditorCurveData = PhysicsControl.TorqueCurve;
  EngineSetup.MaxTorque = PhysicsControl.MaxTorque;
  EngineSetup.MaxRPM = PhysicsControl.MaxRPM;
  EngineSetup.EngineIdleRPM = PhysicsControl.IdleRPM;
  EngineSetup.EngineBrakeEffect = PhysicsControl.BrakeEffect;
  EngineSetup.EngineRevUpMOI = PhysicsControl.RevUpMOI;
  EngineSetup.EngineRevDownRate = PhysicsControl.RevDownRate;
  auto& DifferentialSetup = VehicleMovComponent.DifferentialSetup;
  DifferentialSetup.DifferentialType = (EVehicleDifferential)PhysicsControl.DifferentialType;
  DifferentialSetup.FrontRearSplit = PhysicsControl.FrontRearSplit;
  auto& TransmissionSetup = VehicleMovComponent.TransmissionSetup;
  TransmissionSetup.bUseAutomaticGears = PhysicsControl.bUseAutomaticGears;
  TransmissionSetup.GearChangeTime = PhysicsControl.GearChangeTime;
  TransmissionSetup.FinalRatio = PhysicsControl.FinalRatio;
  TransmissionSetup.ForwardGearRatios = PhysicsControl.ForwardGearRatios;
  TransmissionSetup.ReverseGearRatios = PhysicsControl.ReverseGearRatios;
  TransmissionSetup.ChangeUpRPM = PhysicsControl.ChangeUpRPM;
  TransmissionSetup.ChangeDownRPM = PhysicsControl.ChangeDownRPM;
  TransmissionSetup.TransmissionEfficiency = PhysicsControl.TransmissionEfficiency;
  VehicleMovComponent.Mass = PhysicsControl.Mass;
  VehicleMovComponent.DragCoefficient = PhysicsControl.DragCoefficient;
  SetCenterOfMass(VehicleMovComponent, PhysicsControl);
  VehicleMovComponent.ChassisWidth = PhysicsControl.ChassisWidth;
  VehicleMovComponent.ChassisHeight = PhysicsControl.ChassisHeight;
  VehicleMovComponent.DownforceCoefficient = PhysicsControl.DownforceCoefficient;
  VehicleMovComponent.DragArea = PhysicsControl.DragArea;
  VehicleMovComponent.InertiaTensorScale = PhysicsControl.InertiaTensorScale;
  VehicleMovComponent.SleepThreshold = PhysicsControl.SleepThreshold;
  VehicleMovComponent.SleepSlopeLimit = PhysicsControl.SleepSlopeLimit;
  auto& SteeringSetup = VehicleMovComponent.SteeringSetup;
  SteeringSetup.SteeringCurve.EditorCurveData = PhysicsControl.SteeringCurve;

  // Recreate first: it rebuilds the wheels from the wheel class defaults.
  VehicleMovComponent.RecreatePhysicsState();
  ResetConstraints();
  ApplyWheelPhysicsControl();

  auto* Recorder = UCarlaStatics::GetRecorder(GetWorld());
  if (Recorder && Recorder->IsEnabled())
  {
    Recorder->AddPhysicsControl(*this);
  }

  // Update physics in the Ackermann Controller
  AckermannController.UpdateVehiclePhysics(this);
}

void ACarlaWheeledVehicle::ApplyWheelPhysicsControl()
{
  UChaosWheeledVehicleMovementComponent* Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement == nullptr)
  {
    return;
  }
  const TArray<FWheelPhysicsControl>& WheelControls = LastPhysicsControl.Wheels;
  const int32 WheelCount = WheelControls.Num();
  if (WheelCount == 0)
  {
    return;
  }
  if (WheelCount != Movement->WheelSetups.Num())
  {
    UE_LOG(LogCarla, Error,
        TEXT("%s: %d WheelPhysicsControl for %d wheels, skipping wheel setup"),
        *GetName(), WheelCount, Movement->WheelSetups.Num());
    return;
  }
  // The runtime wheel instances only exist once the Chaos vehicle finishes its
  // physics initialization (after the spawn frame): applying wheel physics
  // before that would index an empty array and crash the server.
  if (Movement->Wheels.Num() < WheelCount)
  {
    UE_LOG(LogCarla, Warning,
        TEXT("ApplyVehiclePhysicsControl: wheels not initialized yet on %s, skipping wheel setup"),
        *GetName());
    return;
  }

  for (int32 i = 0; i < WheelCount; ++i)
  {
    const FWheelPhysicsControl& In = WheelControls[i];
    UChaosVehicleWheel* OutPtr = Movement->Wheels[i];
    check(OutPtr != nullptr);
    UChaosVehicleWheel& Out = *OutPtr;

    Movement->SetWheelRadius(i, In.WheelRadius);
    Movement->SetWheelFrictionMultiplier(i, In.FrictionForceMultiplier);
    Movement->SetWheelMaxSteerAngle(i, In.MaxSteerAngle);
    Movement->SetWheelMaxBrakeTorque(i, In.MaxBrakeTorque);
    Movement->SetWheelHandbrakeTorque(i, In.MaxHandBrakeTorque);
    Movement->SetABSEnabled(i, In.bABSEnabled);
    Movement->SetTractionControlEnabled(i, In.bTractionControlEnabled);
    Movement->SetAffectedByBrake(i, In.bAffectedByBrake);
    Movement->SetAffectedByHandbrake(i, In.bAffectedByHandbrake);
    Movement->SetAffectedBySteering(i, In.bAffectedBySteering);
    // On a front or rear axle the differential decides which wheels are
    // driven: only let the control take power away from one of them.
    if (!In.bAffectedByEngine || Out.AxleType == EAxleType::Undefined)
    {
      Movement->SetAffectedByEngine(i, In.bAffectedByEngine);
    }
    Movement->SetTorqueCombineMethod(In.ExternalTorqueCombineMethod, i);

    // get_physics_control reads these back, and the wheel trace reads the
    // sweep settings from the wheel object every tick.
    Out.WheelRadius = In.WheelRadius;
    Out.FrictionForceMultiplier = In.FrictionForceMultiplier;
    Out.MaxSteerAngle = In.MaxSteerAngle;
    Out.MaxBrakeTorque = In.MaxBrakeTorque;
    Out.MaxHandBrakeTorque = In.MaxHandBrakeTorque;
    Out.bABSEnabled = In.bABSEnabled;
    Out.bTractionControlEnabled = In.bTractionControlEnabled;
    Out.bAffectedByBrake = In.bAffectedByBrake;
    Out.bAffectedByHandbrake = In.bAffectedByHandbrake;
    Out.bAffectedBySteering = In.bAffectedBySteering;
    Out.bAffectedByEngine = In.bAffectedByEngine;
    Out.ExternalTorqueCombineMethod = In.ExternalTorqueCombineMethod;
    Out.SweepShape =
      LastPhysicsControl.UseSweepWheelCollision ?
      ESweepShape::Spherecast :
      ESweepShape::Raycast;
    Out.SweepType = In.SweepType;
  }
}

void ACarlaWheeledVehicle::SetCenterOfMass(UChaosWheeledVehicleMovementComponent& VehicleMovComponent, const FVehiclePhysicsControl& PhysicsControl)
{
  if (VehicleMovComponent.bEnableCenterOfMassOverride)
  {
    VehicleMovComponent.CenterOfMassOverride = PhysicsControl.CenterOfMass;
  }
  else {
    auto PrimitiveComponentPtr = Cast<UPrimitiveComponent>(VehicleMovComponent.UpdatedComponent);
    check(PrimitiveComponentPtr != nullptr);
    auto& PrimitiveComponent = *PrimitiveComponentPtr;
    PrimitiveComponent.BodyInstance.COMNudge = PhysicsControl.CenterOfMass;
  }
}

void ACarlaWheeledVehicle::ApplyVehicleControl(const FVehicleControl& Control, EVehicleInputPriority Priority)
{
  if (bAckermannControlActive) {
    AckermannController.Reset();
  }
  bAckermannControlActive = false;

  if (InputControl.Priority <= Priority)
  {
    InputControl.Control = Control;
    InputControl.Priority = Priority;
  }
}

void ACarlaWheeledVehicle::ApplyVehicleAckermannControl(const FVehicleAckermannControl& AckermannControl, EVehicleInputPriority Priority)
{
  bAckermannControlActive = true;
  LastAppliedAckermannControl = AckermannControl;
  AckermannController.SetTargetPoint(AckermannControl);
}

void ACarlaWheeledVehicle::ActivateVelocityControl(const FVector& Velocity)
{
  VelocityControl->Activate(Velocity);
}

void ACarlaWheeledVehicle::DeactivateVelocityControl()
{
  VelocityControl->Deactivate();
}

void ACarlaWheeledVehicle::ActivateAccelerationControl(const FVector& Acceleration)
{
  // tier4: with a jerk limit or lag set, keep the shaping state across commands.
  // Otherwise re-activate on every command, as upstream does.
  if (AccelerationControl->IsActive() && AccelerationControl->IsShapingEnabled())
  {
    AccelerationControl->SetTargetAcceleration(Acceleration);
  }
  else
  {
    AccelerationControl->Activate(Acceleration);
  }
}

void ACarlaWheeledVehicle::DeactivateAccelerationControl()
{
  AccelerationControl->Deactivate();
}

void ACarlaWheeledVehicle::SetAccelerationControlJerkLimit(float JerkLimitPosMps3, float JerkLimitNegMps3)
{
  if (AccelerationControl == nullptr)
  {
    return;
  }
  // Convert m/s^3 -> cm/s^3.
  const float PosCmps3 = JerkLimitPosMps3 * 100.0f;
  const float NegCmps3 = JerkLimitNegMps3 * 100.0f;
  AccelerationControl->SetJerkLimit(PosCmps3, NegCmps3);
}

void ACarlaWheeledVehicle::SetAccelerationControlFirstOrderLagTau(float InTauS)
{
  if (AccelerationControl == nullptr)
  {
    return;
  }
  AccelerationControl->SetFirstOrderLagTau(InTauS);
}

void ACarlaWheeledVehicle::ApplyVehicleAccelerationControl(float LongitudinalAccelerationMps2, float Steer, float SteerSpeed)
{
  if (bAckermannControlActive)
  {
    AckermannController.Reset();
  }
  bAckermannControlActive = false;
  VelocityControl->Deactivate();

  // Longitudinal: acceleration from control_cmd [m/s^2] -> Unreal uses cm/s^2
  const FVector AccelerationCmps2(LongitudinalAccelerationMps2 * 100.0f, 0.0f, 0.0f);
  ActivateAccelerationControl(AccelerationCmps2);

  InputControl.Control.Steer = Steer;
  InputControl.Priority = EVehicleInputPriority::User;
}

FVehicleTelemetryData ACarlaWheeledVehicle::GetVehicleTelemetryData() const
{
  FVehicleTelemetryData TelemetryData{};

  auto *VehicleMovComponent = GetChaosWheeledVehicleMovementComponent();
  check(VehicleMovComponent != nullptr);

  // Vehicle-level telemetry. Forward speed is reported by the engine in cm/s
  // and converted to m/s. Aerodynamic drag is intentionally not exposed: the
  // Chaos plugin's DebugDragMagnitude is declared but never written, so any
  // value reported here would be a permanent zero.
  TelemetryData.Speed = GetVehicleForwardSpeed() / 100.0f;
  TelemetryData.Steer = LastAppliedControl.Steer;
  TelemetryData.Throttle = LastAppliedControl.Throttle;
  TelemetryData.Brake = LastAppliedControl.Brake;
  TelemetryData.EngineRPM = VehicleMovComponent->GetEngineRotationSpeed();
  TelemetryData.Gear = GetVehicleCurrentGear();

  // Per-wheel telemetry. UChaosWheeledVehicleMovementComponent::GetWheelState
  // returns live solver slip data; UChaosVehicleWheel::GetWheelAngularVelocity
  // is also live. The legacy PhysX surface (tire load, torque, long/lat force)
  // has no Chaos equivalent that is populated in stock UE 5.5 (the Debug*
  // UPROPERTYs on UChaosVehicleWheel are never written by the plugin), so it
  // is omitted here rather than reported as silent zeros.
  const int32 WheelCount = VehicleMovComponent->Wheels.Num();
  TelemetryData.Wheels.Reserve(WheelCount);
  for (int32 i = 0; i < WheelCount; ++i)
  {
    FWheelTelemetryData WheelData{};
    // VehicleMovComponent->Wheels is TArray<TObjectPtr<UChaosVehicleWheel>>;
    // bind to a reference so TObjectPtr's implicit conversion handles the
    // dereferences below (raw `auto *` does not deduce through TObjectPtr).
    const auto &Wheel = VehicleMovComponent->Wheels[i];
    if (!Wheel)
    {
      TelemetryData.Wheels.Add(WheelData);
      continue;
    }

    const FWheelStatus &WheelStatus = VehicleMovComponent->GetWheelState(i);

    WheelData.LatSlip = FMath::RadiansToDegrees(WheelStatus.SlipAngle);
    WheelData.LongSlip = WheelStatus.SlipMagnitude;
    WheelData.Omega = Wheel->GetWheelAngularVelocity();

    TelemetryData.Wheels.Add(WheelData);
  }

  return TelemetryData;
}


void ACarlaWheeledVehicle::ShowDebugTelemetry(bool Enabled)
{
  if (GetWorld()->GetFirstPlayerController())
  {
    ACarlaHUD* hud = Cast<ACarlaHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
    if (hud) {

    // Set/Unset the car movement component in HUD to show the temetry
    if (Enabled) {
      hud->AddDebugVehicleForTelemetry(GetChaosWheeledVehicleMovementComponent());
    }
    else{
      if (hud->DebugVehicle == GetChaosWheeledVehicleMovementComponent()) {
      hud->AddDebugVehicleForTelemetry(nullptr);
      //GetChaosWheeledVehicleMovementComponent()->StopTelemetry();
      }
    }

    }
    else {
    UE_LOG(LogCarla, Warning, TEXT("ACarlaWheeledVehicle::ShowDebugTelemetry:: Cannot find HUD for debug info"));
    }
  }
}

void ACarlaWheeledVehicle::SetVehicleLightState(const FVehicleLightState& LightState)
{
  if (LightState.Position != InputControl.LightState.Position ||
    LightState.LowBeam != InputControl.LightState.LowBeam ||
    LightState.HighBeam != InputControl.LightState.HighBeam ||
    LightState.Brake != InputControl.LightState.Brake ||
    LightState.RightBlinker != InputControl.LightState.RightBlinker ||
    LightState.LeftBlinker != InputControl.LightState.LeftBlinker ||
    LightState.Reverse != InputControl.LightState.Reverse ||
    LightState.Fog != InputControl.LightState.Fog ||
    LightState.Interior != InputControl.LightState.Interior ||
    LightState.Special1 != InputControl.LightState.Special1 ||
    LightState.Special2 != InputControl.LightState.Special2)
  {
    InputControl.LightState = LightState;
    // The blueprint reads its per-group intensity variables (already scaled
    // once at spawn in ActivateVehicleLightComponents) and re-pushes them
    // from a looping timer, so no per-call component rescale is needed here
    // -- it would race that timer anyway.
    RefreshLightState(LightState);
    // RefreshLightState's own graph is opaque to C++ and may repush its own
    // (weak, authored) emissive values for whichever groups just toggled --
    // reapply the Light Defaults saved values after it so ours win.
    ApplyVehicleLightDefaultsForCurrentState();
  }
}

void ACarlaWheeledVehicle::SetFailureState(const carla::rpc::VehicleFailureState& InFailureState)
{
  FailureState = InFailureState;
}

void ACarlaWheeledVehicle::SetCarlaMovementComponent(UBaseCarlaMovementComponent* NewBaseMovementComponent)
{
  if (BaseMovementComponent)
  {
    BaseMovementComponent->DestroyComponent();
  }
  BaseMovementComponent = NewBaseMovementComponent;
}

// The pose the animation blueprint shows for a wheel, or the rest pose if the
// blueprint does not derive from UCarlaVehicleAnimationInstance.
static FWheelAnimationData GetShownWheelPose(const USkeletalMeshComponent &Mesh, int32 WheelIndex)
{
  const auto *VehicleAnim = Cast<UCarlaVehicleAnimationInstance>(Mesh.GetAnimInstance());
  const FWheelAnimationData *Pose = VehicleAnim != nullptr ? VehicleAnim->GetWheelPose(WheelIndex) : nullptr;
  return Pose != nullptr ? *Pose : FWheelAnimationData{NAME_None, FRotator::ZeroRotator, FVector::ZeroVector};
}

void ACarlaWheeledVehicle::SetWheelSteerDirection(EVehicleWheelLocation WheelLocation, float AngleInDeg)
{
  if (FWheelAnimationData *Pose = GetOverriddenWheelPoseOrWarn(WheelLocation, TEXT("SetWheelSteerDirection")))
  {
    Pose->RotOffset.Yaw = AngleInDeg;
  }
}

float ACarlaWheeledVehicle::GetWheelSteerAngle(EVehicleWheelLocation WheelLocation)
{
  // EVehicleWheelLocation is the index into the movement component's wheel
  // arrays: UChaosVehicleWheel::WheelIndex is documented as "our index in the
  // vehicle's (and setup's) wheels array".
  const int32 WheelIndex = static_cast<int32>(WheelLocation);
  return HasWheelOrWarn(WheelIndex, TEXT("GetWheelSteerAngle")) ? GetWheelPose(WheelIndex).RotOffset.Yaw : 0.0F;
}

void ACarlaWheeledVehicle::SetWheelPitchAngle(EVehicleWheelLocation WheelLocation, float AngleInDeg)
{
  if (FWheelAnimationData *Pose = GetOverriddenWheelPoseOrWarn(WheelLocation, TEXT("SetWheelPitchAngle")))
  {
    Pose->RotOffset.Pitch = AngleInDeg;
  }
}

float ACarlaWheeledVehicle::GetWheelPitchAngle(EVehicleWheelLocation WheelLocation)
{
  const int32 WheelIndex = static_cast<int32>(WheelLocation);
  return HasWheelOrWarn(WheelIndex, TEXT("GetWheelPitchAngle")) ? GetWheelPose(WheelIndex).RotOffset.Pitch : 0.0F;
}

void ACarlaWheeledVehicle::SetWheelSuspensionOffset(EVehicleWheelLocation WheelLocation, float OffsetInCm)
{
  if (FWheelAnimationData *Pose = GetOverriddenWheelPoseOrWarn(WheelLocation, TEXT("SetWheelSuspensionOffset")))
  {
    // The same offset Chaos applies.
    Pose->LocOffset = -GetWheelSuspensionAxis(static_cast<int32>(WheelLocation)) * OffsetInCm;
  }
}

float ACarlaWheeledVehicle::GetWheelSuspensionOffset(EVehicleWheelLocation WheelLocation)
{
  const int32 WheelIndex = static_cast<int32>(WheelLocation);
  if (!HasWheelOrWarn(WheelIndex, TEXT("GetWheelSuspensionOffset")))
  {
    return 0.0F;
  }
  return -FVector::DotProduct(GetWheelPose(WheelIndex).LocOffset, GetWheelSuspensionAxis(WheelIndex));
}

void ACarlaWheeledVehicle::SetWheelAnimationOverride(bool bEnabled)
{
  if (bEnabled && !HasCarlaVehicleAnimation())
  {
    UE_LOG(LogCarla, Warning,
        TEXT("SetWheelAnimationOverride: the animation blueprint of %s does not derive from CarlaVehicleAnimationInstance."),
        *GetName());
    return;
  }
  if (bEnabled && !bWheelAnimationOverridden)
  {
    // Start from the latest pose, so the wheels do not jump.
    OverriddenWheelPoses = GetWheelPoses();
  }
  else if (!bEnabled)
  {
    if (bWheelAnimationOverridden && IsSimulatedByChaos())
    {
      SeedChaosWheels(OverriddenWheelPoses);
    }
    OverriddenWheelPoses.Reset();
  }
  bWheelAnimationOverridden = bEnabled;
}

FWheelAnimationData *ACarlaWheeledVehicle::GetOverriddenWheelPoseOrWarn(
    EVehicleWheelLocation WheelLocation,
    const TCHAR *Caller)
{
  const int32 WheelIndex = static_cast<int32>(WheelLocation);
  if (!HasWheelOrWarn(WheelIndex, Caller))
  {
    return nullptr;
  }
  if (!bWheelAnimationOverridden)
  {
    UE_LOG(LogCarla, Warning,
        TEXT("%s: the wheel animation override is off on %s; enable it first."),
        Caller, *GetName());
    return nullptr;
  }
  return OverriddenWheelPoses.IsValidIndex(WheelIndex) ? &OverriddenWheelPoses[WheelIndex] : nullptr;
}

FWheelAnimationData ACarlaWheeledVehicle::GetWheelPose(int32 WheelIndex) const
{
  if (bWheelAnimationOverridden && OverriddenWheelPoses.IsValidIndex(WheelIndex))
  {
    return OverriddenWheelPoses[WheelIndex];
  }
  if (const UChaosVehicleWheel *Wheel = GetSimulatedWheel(WheelIndex))
  {
    // The pose Chaos draws, in degrees and cm.
    const FRotator Rotation(Wheel->GetRotationAngle(), Wheel->GetSteerAngle(), 0.0f);
    const FVector Offset = -GetWheelSuspensionAxis(WheelIndex) * Wheel->GetSuspensionOffset();
    return FWheelAnimationData{NAME_None, Rotation, Offset};
  }
  return GetShownWheelPose(*GetMesh(), WheelIndex);
}

FWheelAnimationData ACarlaWheeledVehicle::GetDrawnWheelPose(int32 WheelIndex) const
{
  const auto *VehicleAnim = Cast<UCarlaVehicleAnimationInstance>(GetMesh()->GetAnimInstance());
  if (const FWheelAnimationData *Pose = VehicleAnim != nullptr ? VehicleAnim->GetWheelPose(WheelIndex) : nullptr)
  {
    return *Pose;
  }
  return GetWheelPose(WheelIndex);
}

FVector ACarlaWheeledVehicle::GetWheelWorldLocation(int32 WheelIndex) const
{
  const UChaosWheeledVehicleMovementComponent *Movement = GetChaosWheeledVehicleMovementComponent();
  const UChaosVehicleWheel *Wheel = GetWheelDefaults(WheelIndex);
  if (Movement == nullptr || Wheel == nullptr)
  {
    return GetActorLocation();
  }
  // The resting position Chaos gives the wheel (GetWheelRestingPosition, which
  // is protected), relative to the root body, plus the drawn suspension travel.
  const FChaosWheelSetup &WheelSetup = Movement->WheelSetups[WheelIndex];
  const FVector Local =
      Movement->LocateBoneOffset(WheelSetup.BoneName, Wheel->Offset + WheelSetup.AdditionalOffset) +
      GetWheelPose(WheelIndex).LocOffset;
  const FBodyInstance *RootBody = GetMesh()->GetBodyInstance();
  const FTransform RootBodyTransform = RootBody != nullptr && RootBody->IsValidBodyInstance() ?
      RootBody->GetUnrealWorldTransform() : GetMesh()->GetComponentTransform();
  return RootBodyTransform.TransformPosition(Local);
}

TArray<FWheelAnimationData> ACarlaWheeledVehicle::GetWheelPoses() const
{
  TArray<FWheelAnimationData> Poses;
  for (int32 i = 0; HasWheel(i); ++i)
  {
    Poses.Add(GetWheelPose(i));
  }
  return Poses;
}

void ACarlaWheeledVehicle::SeedChaosWheels(const TArray<FWheelAnimationData> &Poses)
{
  UChaosWheeledVehicleMovementComponent *Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement == nullptr)
  {
    return;
  }
  // SetSnapshot dereferences the physics vehicle, created with this output.
  TUniquePtr<FPhysicsVehicleOutput> &Output = Movement->PhysicsVehicleOutput();
  if (!Output.IsValid() || Output->Wheels.Num() != Poses.Num())
  {
    return;
  }
  FWheeledSnaphotData Snapshot;
  Movement->GetBaseSnapshot(Snapshot);
  TArray<float> Spins;
  const float ForwardSpeed = FVector::DotProduct(Snapshot.LinearVelocity, GetActorForwardVector());
  for (int32 i = 0; i < Poses.Num(); ++i)
  {
    // Chaos recomputes steering and suspension every step; only the spin carries over.
    const FWheelAnimationData &Pose = Poses[i];
    const float Radius = GetWheelRadius(i);
    FWheelSnapshot &Wheel = Snapshot.WheelSnapshots.AddDefaulted_GetRef();
    Wheel.WheelRotationAngle = Pose.RotOffset.Pitch;
    Wheel.SteeringAngle = Pose.RotOffset.Yaw;
    Wheel.SuspensionOffset = -FVector::DotProduct(Pose.LocOffset, GetWheelSuspensionAxis(i));
    Wheel.WheelRadius = Radius;
    Wheel.WheelAngularVelocity = Radius > UE_KINDA_SMALL_NUMBER ? ForwardSpeed / Radius : 0.0f;
    // What is drawn until the next physics step, which a sleeping vehicle skips.
    Output->Wheels[i].AngularPosition = FMath::DegreesToRadians(-Wheel.WheelRotationAngle);
    Spins.Add(Wheel.WheelRotationAngle);
  }
  Movement->SetSnapshot(Snapshot);
  ArmSeedRewrite(MoveTemp(Spins));
}

void ACarlaWheeledVehicle::ArmSeedRewrite(TArray<float> Spins)
{
  DisarmSeedRewrite();
  FPhysScene *Scene = GetWorld() != nullptr ? GetWorld()->GetPhysicsScene() : nullptr;
  if (Scene == nullptr)
  {
    return;
  }
  PendingSeedSpins = MoveTemp(Spins);
  bSeedStepStarted = false;
  SeedPreTickHandle = Scene->OnPhysScenePreTick.AddUObject(this, &ACarlaWheeledVehicle::OnSeedPreTick);
  SeedPostTickHandle = Scene->OnPhysScenePostTick.AddUObject(this, &ACarlaWheeledVehicle::OnSeedPostTick);
}

void ACarlaWheeledVehicle::DisarmSeedRewrite()
{
  FPhysScene *Scene = GetWorld() != nullptr ? GetWorld()->GetPhysicsScene() : nullptr;
  if (Scene != nullptr)
  {
    Scene->OnPhysScenePreTick.Remove(SeedPreTickHandle);
    Scene->OnPhysScenePostTick.Remove(SeedPostTickHandle);
  }
  SeedPreTickHandle.Reset();
  SeedPostTickHandle.Reset();
  PendingSeedSpins.Reset();
}

void ACarlaWheeledVehicle::OnSeedPreTick(FPhysScene_Chaos *Scene, float DeltaSeconds)
{
  // This step is the first to start from the seeded wheels.
  bSeedStepStarted = true;
}

void ACarlaWheeledVehicle::OnSeedPostTick(FChaosScene *Scene)
{
  if (!bSeedStepStarted)
  {
    return;
  }
  UChaosWheeledVehicleMovementComponent *Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement != nullptr)
  {
    TUniquePtr<FPhysicsVehicleOutput> &Output = Movement->PhysicsVehicleOutput();
    if (Output.IsValid() && Output->Wheels.Num() == PendingSeedSpins.Num())
    {
      for (int32 i = 0; i < PendingSeedSpins.Num(); ++i)
      {
        Output->Wheels[i].AngularPosition = FMath::DegreesToRadians(-PendingSeedSpins[i]);
      }
    }
  }
  DisarmSeedRewrite();
}

bool ACarlaWheeledVehicle::HasWheelOrWarn(int32 WheelIndex, const TCHAR *Caller) const
{
  if (!HasWheel(WheelIndex))
  {
    // Reachable from the client, a bike asked for BL_Wheel say, so it must not
    // bring the server down.
    UE_LOG(LogCarla, Warning,
        TEXT("%s: wheel location %d does not exist on %s."),
        Caller, WheelIndex, *GetName());
    return false;
  }
  return true;
}

bool ACarlaWheeledVehicle::HasWheel(int32 WheelIndex) const
{
  const UChaosWheeledVehicleMovementComponent *Movement = GetChaosWheeledVehicleMovementComponent();
  return Movement != nullptr && Movement->WheelSetups.IsValidIndex(WheelIndex);
}

bool ACarlaWheeledVehicle::HasCarlaVehicleAnimation() const
{
  return Cast<UCarlaVehicleAnimationInstance>(GetMesh()->GetAnimInstance()) != nullptr;
}

const UChaosVehicleWheel *ACarlaWheeledVehicle::GetSimulatedWheel(int32 WheelIndex) const
{
  // Non-const: the engine does not mark PhysicsVehicleOutput() const.
  UChaosWheeledVehicleMovementComponent *Movement = GetChaosWheeledVehicleMovementComponent();
  if (!IsSimulatedByChaos() || Movement == nullptr ||
      !Movement->Wheels.IsValidIndex(WheelIndex))
  {
    return nullptr;
  }
  // The wheel getters read the async solver's output and check() that it is
  // there; it only exists once the physics state has been created and stepped.
  const UChaosVehicleWheel *Wheel = Movement->Wheels[WheelIndex];
  const TUniquePtr<FPhysicsVehicleOutput> &VehicleOutput = Movement->PhysicsVehicleOutput();
  if (Wheel == nullptr || !VehicleOutput.IsValid() ||
      !VehicleOutput->Wheels.IsValidIndex(Wheel->WheelIndex))
  {
    return nullptr;
  }
  return Wheel;
}

bool ACarlaWheeledVehicle::IsSimulatedByChaos() const
{
  return bPhysicsEnabled &&
      (BaseMovementComponent == nullptr || BaseMovementComponent->IsA<UDefaultMovementComponent>());
}

float ACarlaWheeledVehicle::GetWheelRadius(int32 WheelIndex) const
{
  if (LastPhysicsControl.Wheels.IsValidIndex(WheelIndex))
  {
    return LastPhysicsControl.Wheels[WheelIndex].WheelRadius;
  }
  const UChaosVehicleWheel *Wheel = GetWheelDefaults(WheelIndex);
  return Wheel != nullptr ? Wheel->WheelRadius : 0.0f;
}

FVector ACarlaWheeledVehicle::GetWheelSuspensionAxis(int32 WheelIndex) const
{
  if (LastPhysicsControl.Wheels.IsValidIndex(WheelIndex))
  {
    return LastPhysicsControl.Wheels[WheelIndex].SuspensionAxis.GetSafeNormal();
  }
  const UChaosVehicleWheel *Wheel = GetWheelDefaults(WheelIndex);
  return Wheel != nullptr ? Wheel->SuspensionAxis.GetSafeNormal() : FVector::DownVector;
}

const UChaosVehicleWheel *ACarlaWheeledVehicle::GetWheelDefaults(int32 WheelIndex) const
{
  // For when a rejected physics control left fewer entries than wheels.
  const UChaosWheeledVehicleMovementComponent *Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement == nullptr || !Movement->WheelSetups.IsValidIndex(WheelIndex))
  {
    return nullptr;
  }
  return Movement->WheelSetups[WheelIndex].WheelClass.GetDefaultObject();
}

void ACarlaWheeledVehicle::SetSimulatePhysics(bool enabled) {
  if (!GetCarlaMovementComponent<UDefaultMovementComponent>())
  {
    return;
  }

  UChaosWheeledVehicleMovementComponent* Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement)
  {
    check(Movement != nullptr);

    if (bPhysicsEnabled == enabled)
      return;

    // Rebuilding the physics state resets the poses drawn.
    const TArray<FWheelAnimationData> ShownPoses = GetWheelPoses();

    SetActorEnableCollision(true);
    UPrimitiveComponent* RootPrimitive =
      Cast<UPrimitiveComponent>(GetRootComponent());
    RootPrimitive->SetSimulatePhysics(enabled);
    RootPrimitive->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

      if (enabled)
      {
        Movement->SetUpdatedComponent(GetMesh());
        Movement->RecreatePhysicsState();
        ApplyWheelPhysicsControl();
        // The recreated body starts asleep and Chaos vehicle input does not
        // wake it, leaving the drivetrain permanently inert (full throttle,
        // zero motion) until something else wakes the body. Wake it here.
        RootPrimitive->WakeAllRigidBodies();
        SeedChaosWheels(ShownPoses);
      }
      else
      {
        Movement->DestroyPhysicsState();
      }
    bPhysicsEnabled = enabled;

    ResetConstraints();
  }
}

void ACarlaWheeledVehicle::ResetConstraints()
{
  // Rebuilding the constraints must not change which doors are open: physics
  // control and physics toggles (the replayer's among them) call this. A
  // Multi-GPU mirror's doors are posed from the primary instead, next frame.
  const uint8 OpenDoors = OpenDoorMask;
  const UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  const bool bReopen = (Episode == nullptr) || Episode->bIsPrimaryServer;
  TGuardValue<bool> NoRecording(bResettingDoorConstraints, true);
  for (int i = 0; i < ConstraintsComponents.Num(); i++)
  {
    OpenDoorPhys(EVehicleDoor(i));
  }
  for (int i = 0; i < ConstraintsComponents.Num(); i++)
  {
    CloseDoorPhys(EVehicleDoor(i));
  }
  for (int i = 0; bReopen && (i < ConstraintsComponents.Num()); i++)
  {
    if ((OpenDoors & (1u << i)) != 0u)
    {
      OpenDoorPhys(EVehicleDoor(i));
    }
  }
}

FVector ACarlaWheeledVehicle::GetVelocity() const
{
  if (bHasMirroredMotion)
  {
    return MirroredVelocity;
  }
  return BaseMovementComponent->GetVelocity();
}

void ACarlaWheeledVehicle::SetMirroredMotion(const FVector &Velocity, float ForwardSpeed)
{
  bHasMirroredMotion = true;
  MirroredVelocity = Velocity;
  MirroredForwardSpeed = ForwardSpeed;
}

void ACarlaWheeledVehicle::ClearMirroredMotion()
{
  bHasMirroredMotion = false;
  MirroredVelocity = FVector::ZeroVector;
  MirroredForwardSpeed = 0.0f;
}

void ACarlaWheeledVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  ShowDebugTelemetry(false);
  DisarmSeedRewrite();
  Super::EndPlay(EndPlayReason);
  RemoveReferenceToManager();
}

void ACarlaWheeledVehicle::OpenDoor(const EVehicleDoor DoorIdx) {
  if (int(DoorIdx) >= ConstraintsComponents.Num() && DoorIdx != EVehicleDoor::All) {
    UE_LOG(LogTemp, Warning, TEXT("This door is not configured for this car."));
    return;
  }

  if (DoorIdx == EVehicleDoor::All) {
    for (int i = 0; i < ConstraintsComponents.Num(); i++)
    {
      OpenDoorPhys(EVehicleDoor(i));
    }
    return;
  }

  OpenDoorPhys(DoorIdx);
}

void ACarlaWheeledVehicle::CloseDoor(const EVehicleDoor DoorIdx) {
  if (int(DoorIdx) >= ConstraintsComponents.Num() && DoorIdx != EVehicleDoor::All) {
    UE_LOG(LogTemp, Warning, TEXT("This door is not configured for this car."));
    return;
  }

  if (DoorIdx == EVehicleDoor::All) {
    for (int i = 0; i < ConstraintsComponents.Num(); i++)
    {
      CloseDoorPhys(EVehicleDoor(i));
    }
    return;
  }

  CloseDoorPhys(DoorIdx);
}

void ACarlaWheeledVehicle::OpenDoorPhys(const EVehicleDoor DoorIdx)
{
  UPhysicsConstraintComponent* Constraint = ConstraintsComponents[static_cast<int>(DoorIdx)];
  UPrimitiveComponent* DoorComponent = ConstraintDoor[Constraint];
  DoorComponent->DetachFromComponent(
    FDetachmentTransformRules(EDetachmentRule::KeepWorld, false));
  FTransform DoorInitialTransform =
    DoorComponentsTransform[DoorComponent] * GetActorTransform();
  DoorComponent->SetWorldTransform(DoorInitialTransform);
  DoorComponent->SetSimulatePhysics(true);
  DoorComponent->SetCollisionProfileName(TEXT("BlockAll"));
  float AngleLimit = Constraint->ConstraintInstance.GetAngularSwing1Limit();
  FRotator AngularRotationOffset = Constraint->ConstraintInstance.AngularRotationOffset;

  if (Constraint->ConstraintInstance.AngularRotationOffset.Yaw < 0.0f)
  {
    AngleLimit = -AngleLimit;
  }
  Constraint->SetAngularOrientationTarget(FRotator(0, AngleLimit, 0));
  Constraint->SetAngularDriveParams(DoorOpenStrength, 1.0, 0.0);

  Constraint->InitComponentConstraint();

  UPhysicsConstraintComponent** CollisionDisable =
    CollisionDisableConstraints.Find(DoorComponent);
  if (CollisionDisable)
  {
    (*CollisionDisable)->InitComponentConstraint();
  }

  OpenDoorMask |= static_cast<uint8>(1u << static_cast<int>(DoorIdx));
  RecordDoorChange(DoorIdx, true);
}

void ACarlaWheeledVehicle::CloseDoorPhys(const EVehicleDoor DoorIdx)
{
  UPhysicsConstraintComponent* Constraint = ConstraintsComponents[static_cast<int>(DoorIdx)];
  UPrimitiveComponent* DoorComponent = ConstraintDoor[Constraint];
  // Remove the joints before disabling the door body and welding it back
  // into the chassis. Otherwise Chaos can retain a self-referencing edge.
  Constraint->TermComponentConstraint();
  if (UPhysicsConstraintComponent** CollisionDisable = CollisionDisableConstraints.Find(DoorComponent))
  {
    (*CollisionDisable)->TermComponentConstraint();
  }
  FTransform DoorInitialTransform =
    DoorComponentsTransform[DoorComponent] * GetActorTransform();
  DoorComponent->SetSimulatePhysics(false);
  DoorComponent->SetCollisionProfileName(TEXT("NoCollision"));
  DoorComponent->SetWorldTransform(DoorInitialTransform);
  DoorComponent->AttachToComponent(
    GetMesh(), FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
  OpenDoorMask &= static_cast<uint8>(~(1u << static_cast<int>(DoorIdx)));
  RecordDoorChange(DoorIdx, false);
}

TArray<TPair<int32, FTransform>> ACarlaWheeledVehicle::GetOpenDoorPoses() const
{
  TArray<TPair<int32, FTransform>> Poses;
  const FTransform ActorInverseTransform = GetActorTransform().Inverse();
  for (int32 i = 0; i < ConstraintsComponents.Num(); ++i)
  {
    UPrimitiveComponent* const* Door = ConstraintDoor.Find(ConstraintsComponents[i]);
    // OpenDoorPhys detaches a door, CloseDoorPhys attaches it back.
    if ((Door != nullptr) && (*Door != nullptr) && ((*Door)->GetAttachParent() == nullptr))
    {
      Poses.Emplace(i, (*Door)->GetComponentTransform() * ActorInverseTransform);
    }
  }
  return Poses;
}

void ACarlaWheeledVehicle::SetMirroredDoorPoses(const TArray<TPair<int32, FTransform>> &OpenDoors)
{
  OpenDoorMask = 0u;
  for (const TPair<int32, FTransform> &Open : OpenDoors)
  {
    if ((Open.Key >= 0) && (Open.Key < 8))
    {
      OpenDoorMask |= static_cast<uint8>(1u << Open.Key);
    }
  }
  const FTransform ActorTransform = GetActorTransform();
  for (int32 i = 0; i < ConstraintsComponents.Num(); ++i)
  {
    UPrimitiveComponent* const* DoorPtr = ConstraintDoor.Find(ConstraintsComponents[i]);
    UPrimitiveComponent* Door = (DoorPtr != nullptr) ? *DoorPtr : nullptr;
    if (Door == nullptr)
    {
      continue;
    }
    const TPair<int32, FTransform>* Open =
        OpenDoors.FindByPredicate([i](const TPair<int32, FTransform>& Pose) { return Pose.Key == i; });
    const bool bShownOpen = (Door->GetAttachParent() == nullptr);
    if (Open != nullptr)
    {
      if (!bShownOpen)
      {
        Door->DetachFromComponent(FDetachmentTransformRules(EDetachmentRule::KeepWorld, false));
        // What ray casts hit on the primary, where it is a physics body.
        Door->SetCollisionProfileName(TEXT("BlockAll"));
      }
      Door->SetWorldTransform(Open->Value * ActorTransform);
    }
    else if (bShownOpen)
    {
      // As CloseDoorPhys leaves it.
      Door->SetCollisionProfileName(TEXT("NoCollision"));
      Door->SetWorldTransform(DoorComponentsTransform.FindRef(Door) * ActorTransform);
      Door->AttachToComponent(GetMesh(), FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
    }
  }
}

void ACarlaWheeledVehicle::RecordDoorChange(const EVehicleDoor DoorIdx, bool bIsOpen)
{
  if (bResettingDoorConstraints)
  {
    return;
  }
  auto* Recorder = UCarlaStatics::GetRecorder(GetWorld());
  if (Recorder && Recorder->IsEnabled())
  {
    Recorder->AddVehicleDoor(*this, DoorIdx, bIsOpen);
  }
}

void ACarlaWheeledVehicle::ApplyRolloverBehavior()
{
  auto roll = GetVehicleTransform().Rotator().Roll;

  // The angular velocity reduction is applied in 4 stages, to improve its smoothness.
  // Case 4 starts the timer to set the rollover flag, so users are notified.
  switch (RolloverBehaviorTracker) {
  case 0: CheckRollover(roll, std::make_pair(130.0, 230.0));      break;
  case 1: CheckRollover(roll, std::make_pair(140.0, 220.0));      break;
  case 2: CheckRollover(roll, std::make_pair(150.0, 210.0));      break;
  case 3: CheckRollover(roll, std::make_pair(160.0, 200.0));      break;
  case 4:
    GetWorld()->GetTimerManager().SetTimer(TimerHandler, this, &ACarlaWheeledVehicle::SetRolloverFlag, RolloverFlagTime);
    RolloverBehaviorTracker += 1;
    break;
  case 5: break;
  default:
    RolloverBehaviorTracker = 5;
  }

  // In case the vehicle recovers, reset the rollover tracker
  if (RolloverBehaviorTracker > 0 && -30 < roll && roll < 30) {
    RolloverBehaviorTracker = 0;
    FailureState = carla::rpc::VehicleFailureState::None;
  }
}

void ACarlaWheeledVehicle::CheckRollover(const float roll, const std::pair<float, float> threshold_roll) {
  if (threshold_roll.first < roll && roll < threshold_roll.second) {
    auto Root = Cast<UPrimitiveComponent>(GetRootComponent());
    auto AngularVelocity = Root->GetPhysicsAngularVelocityInDegrees();
    Root->SetPhysicsAngularVelocityInDegrees((1 - RolloverBehaviorForce) * AngularVelocity);
    RolloverBehaviorTracker += 1;
  }
}

void ACarlaWheeledVehicle::SetRolloverFlag() {
  // Make sure the vehicle hasn't recovered since the timer started
  if (RolloverBehaviorTracker >= 4) {
    FailureState = carla::rpc::VehicleFailureState::Rollover;
  }
}

carla::rpc::VehicleFailureState ACarlaWheeledVehicle::GetFailureState() const {
  return FailureState;
}

void ACarlaWheeledVehicle::AddReferenceToManager()
{
  const UObject* World = GetWorld();
  TArray<AActor*> ActorsInLevel;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), ActorsInLevel);
  for (AActor* Actor : ActorsInLevel)
  {
    AVegetationManager* Manager = Cast<AVegetationManager>(Actor);
    if (!IsValid(Manager))
      continue;
    Manager->AddVehicle(this);
    return;
  }
}

void ACarlaWheeledVehicle::RemoveReferenceToManager()
{
  const UObject* World = GetWorld();
  TArray<AActor*> ActorsInLevel;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), ActorsInLevel);
  for (AActor* Actor : ActorsInLevel)
  {
    AVegetationManager* Manager = Cast<AVegetationManager>(Actor);
    if (!IsValid(Manager))
      continue;
    Manager->RemoveVehicle(this);
    return;
  }
}

FRotator ACarlaWheeledVehicle::GetPhysicsConstraintAngle(
  UPhysicsConstraintComponent* Component)
{
  return Component->ConstraintInstance.AngularRotationOffset;
}

void ACarlaWheeledVehicle::SetPhysicsConstraintAngle(
  UPhysicsConstraintComponent* Component, const FRotator& NewAngle)
{
  Component->ConstraintInstance.AngularRotationOffset = NewAngle;
}

FPoseSnapshot ACarlaWheeledVehicle::GetWorldTransformedPose()
{
  if(WorldTransformedPose.bIsValid == false)
  {
    SetActorTickEnabled(true);
    GetMesh()->SnapshotPose(WorldTransformedPose);
    MakePoseWorldSpace(GetMesh(), WorldTransformedPose);
  }
  return WorldTransformedPose;
}
