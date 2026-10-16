#include "UBotPhysicsBackend.h"

#include "UBotPhysics.h"

namespace UBot::Physics
{
    TSharedPtr<IUBotPhysicsBackendFactory> FindBackendFactory(FName BackendName)
    {
        if (BackendName.IsNone())
        {
            return nullptr;
        }
        for (const TSharedRef<IUBotPhysicsBackendFactory>& Factory
            : FUBotExtensionRegistry::Get().GetExtensionsAs<IUBotPhysicsBackendFactory>(BackendExtensionPoint))
        {
            if (Factory->GetExtensionName() == BackendName)
            {
                return Factory;
            }
        }
        return nullptr;
    }

    TArray<FName> GetBackendNames()
    {
        TArray<FName> Names;
        for (const TSharedRef<IUBotExtension>& Extension : FUBotExtensionRegistry::Get().GetExtensions(BackendExtensionPoint))
        {
            Names.Add(Extension->GetExtensionName());
        }
        return Names;
    }
}
