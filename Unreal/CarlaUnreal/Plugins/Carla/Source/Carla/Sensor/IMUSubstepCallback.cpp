// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/IMUSubstepCallback.h"

void FIMUSubstepCallback::OnPostSolve_Internal()
{
  const FIMUSubstepInput* In = GetConsumerInput_Internal();
  if (In == nullptr || In->Proxy == nullptr)
  {
    return;
  }

  // Gate on a valid physics-thread API (the proxy must have a live handle).
  if (In->Proxy->GetPhysicsThreadAPI() == nullptr)
  {
    return;
  }

  // Read the solved kinematics from the low-level rigid particle handle.
  // After the constraint solve, GetP()/GetQ() hold this substep's final pose;
  // they only exist on the rigid particle handle, not on the
  // FRigidBodyHandle_Internal proxy API (the engine reads P/Q the same way
  // during post-solve, PhysicsReplicationCache.cpp). Do not read them at
  // PostIntegrate: there they are the prediction before the constraints
  // (e.g. the suspension) are solved, which contains one substep of free fall
  // under gravity while the body is awake and none while it sleeps, so the
  // second difference jumps by about 1 g whenever a slow or resting vehicle
  // goes to sleep or wakes up.
  Chaos::FGeometryParticleHandle* GtHandle = In->Proxy->GetHandle_LowLevel();
  Chaos::FPBDRigidParticleHandle* Rigid =
      (GtHandle != nullptr) ? GtHandle->CastToRigidParticle() : nullptr;
  if (Rigid == nullptr)
  {
    return;
  }

  FIMUSubstepOutput& Out = GetProducerOutputData_Internal();
  FIMUSubstepSample S;
  S.PosP = FVector(Rigid->GetP());
  S.RotQ = FQuat(Rigid->GetQ());
  S.Dt   = static_cast<float>(GetDeltaTime_Internal());
  Out.Samples.Add(S);
}
