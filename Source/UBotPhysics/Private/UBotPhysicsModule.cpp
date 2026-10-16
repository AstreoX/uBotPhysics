#include "UBotPhysics.h"

#include "Modules/ModuleManager.h"
#include "UBotExtensionRegistry.h"

#define LOCTEXT_NAMESPACE "UBotPhysics"

DEFINE_LOG_CATEGORY(LogUBotPhysics);

void FUBotPhysicsModule::StartupModule()
{
    FUBotExtensionRegistry::Get().RegisterExtensionPoint(UBot::Physics::BackendExtensionPoint, UBot::Physics::PackageName,
        LOCTEXT("BackendPoint", "Physics backends that simulate the bodies bound to them (IUBotPhysicsBackendFactory). Experimental."));
}

void FUBotPhysicsModule::ShutdownModule()
{
    FUBotExtensionRegistry::Get().UnregisterAllFromPackage(UBot::Physics::PackageName);
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUBotPhysicsModule, UBotPhysics)
