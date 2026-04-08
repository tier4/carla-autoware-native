#include "CarlaRGLModule.h"
#include "Carla/RGL/IRGLBackend.h"

// RGLBackendImpl will be added in Task 4. For now, module compiles but does not register a backend.
// #ifdef WITH_RGL
// #include "RGLBackendImpl.h"
// #endif

void FCarlaRGLModule::StartupModule()
{
    UE_LOG(LogTemp, Log, TEXT("CarlaRGL module loaded."));
    // Backend registration will be added in Task 4:
    // #ifdef WITH_RGL
    //     FRGLBackendRegistry::Register(new FRGLBackendImpl());
    // #endif
}

void FCarlaRGLModule::ShutdownModule()
{
    FRGLBackendRegistry::Unregister();
}

IMPLEMENT_MODULE(FCarlaRGLModule, CarlaRGL)
