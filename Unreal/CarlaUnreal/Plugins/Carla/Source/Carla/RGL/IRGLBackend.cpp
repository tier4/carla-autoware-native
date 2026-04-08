#include "Carla/RGL/IRGLBackend.h"

IRGLBackend* FRGLBackendRegistry::Instance = nullptr;

void FRGLBackendRegistry::Register(IRGLBackend* Backend)
{
    Instance = Backend;
}

void FRGLBackendRegistry::Unregister()
{
    delete Instance;
    Instance = nullptr;
}

IRGLBackend* FRGLBackendRegistry::Get()
{
    return Instance;
}
