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

    /** Actor or component tag that keeps a component out of every backend's scene. */
    inline const FName NoSceneTag(TEXT("UBotPhysics.NoScene"));
}

class FUBotPhysicsModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
