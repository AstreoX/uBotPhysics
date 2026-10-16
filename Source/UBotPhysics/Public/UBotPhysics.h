#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

UBOTPHYSICS_API DECLARE_LOG_CATEGORY_EXTERN(LogUBotPhysics, Log, All);

namespace UBot::Physics
{
    /** Package name used for extension registrations of this plugin. */
    inline const FName PackageName(TEXT("UBotPhysics"));

    /** Extension point for physics backends (IUBotPhysicsBackendFactory). Experimental. */
    inline const FName BackendExtensionPoint(TEXT("UBotPhysics.Backend"));
}

class FUBotPhysicsModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
