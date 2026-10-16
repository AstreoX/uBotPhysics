#include "UBotPhysicsLibrary.h"

#include "Components/PrimitiveComponent.h"
#include "UBotPhysicsBackend.h"
#include "UBotPhysicsSubsystem.h"

TArray<FName> UUBotPhysicsLibrary::GetPhysicsBackendNames()
{
    return UBot::Physics::GetBackendNames();
}

bool UUBotPhysicsLibrary::GetPhysicsBackendCapabilities(FName Backend, FUBotPhysicsCapabilities& Capabilities)
{
    const TSharedPtr<IUBotPhysicsBackendFactory> Factory = UBot::Physics::FindBackendFactory(Backend);
    if (!Factory.IsValid())
    {
        Capabilities = FUBotPhysicsCapabilities();
        return false;
    }
    Capabilities = Factory->GetCapabilities();
    return true;
}

FName UUBotPhysicsLibrary::GetBodyPhysicsBackend(const UPrimitiveComponent* Component)
{
    const UUBotPhysicsSubsystem* Subsystem = UUBotPhysicsSubsystem::Get(Component);
    return Subsystem != nullptr ? Subsystem->FindBodyBackend(Component) : NAME_None;
}

int64 UUBotPhysicsLibrary::GetDroppedPhysicsSteps(const UObject* WorldContextObject, FName Backend)
{
    const UUBotPhysicsSubsystem* Subsystem = UUBotPhysicsSubsystem::Get(WorldContextObject);
    return Subsystem != nullptr ? Subsystem->GetDroppedSteps(Backend) : 0;
}
