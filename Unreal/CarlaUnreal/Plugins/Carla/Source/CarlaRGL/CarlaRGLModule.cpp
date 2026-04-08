#include "CarlaRGLModule.h"
#include "Carla/RGL/IRGLBackend.h"

DEFINE_LOG_CATEGORY(LogCarlaRGL);

#ifdef WITH_RGL
#include "RGLBackendImpl.h"
#endif

void FCarlaRGLModule::StartupModule()
{
#ifdef WITH_RGL
    FRGLBackendRegistry::Register(new FRGLBackendImpl());
    UE_LOG(LogTemp, Log, TEXT("CarlaRGL: RGL backend registered."));
#else
    UE_LOG(LogTemp, Log, TEXT("CarlaRGL: WITH_RGL not defined, no backend registered."));
#endif
}

void FCarlaRGLModule::ShutdownModule()
{
    FRGLBackendRegistry::Unregister();
}

IMPLEMENT_MODULE(FCarlaRGLModule, CarlaRGL)
