#pragma once
#include "Modules/ModuleManager.h"
#include "Delegates/IDelegateInstance.h"

DECLARE_LOG_CATEGORY_EXTERN(LogCarlaRGL, Log, All);

class FCarlaRGLModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

private:
    // Handle for FWorldDelegates::OnWorldCleanup, registered in StartupModule when WITH_RGL.
    FDelegateHandle WorldCleanupHandle;
};
