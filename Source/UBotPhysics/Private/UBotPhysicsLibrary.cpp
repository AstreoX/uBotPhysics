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

bool UUBotPhysicsLibrary::GetPhysicsSceneStats(const UObject* WorldContextObject, FName Backend, FUBotPhysicsSceneStats& Stats)
{
    const UUBotPhysicsSubsystem* Subsystem = UUBotPhysicsSubsystem::Get(WorldContextObject);
    if (Subsystem == nullptr)
    {
        Stats = FUBotPhysicsSceneStats();
        return false;
    }
    return Subsystem->GetSceneStats(Backend, Stats);
}

EUBotPhysicsBackendStatus UUBotPhysicsLibrary::GetPhysicsBackendStatus(const UObject* WorldContextObject, FName Backend, FText& Error)
{
    const UUBotPhysicsSubsystem* Subsystem = UUBotPhysicsSubsystem::Get(WorldContextObject);
    Error = FText::GetEmpty();
    return Subsystem != nullptr ? Subsystem->GetBackendStatus(Backend, Error) : EUBotPhysicsBackendStatus::Missing;
}

bool UUBotPhysicsLibrary::ResetPhysicsBackend(const UObject* WorldContextObject, FName Backend, FText& Error)
{
    UUBotPhysicsSubsystem* Subsystem = UUBotPhysicsSubsystem::Get(WorldContextObject);
    if (Subsystem == nullptr)
    {
        Error = INVTEXT("No physics subsystem is available.");
        return false;
    }
    return Subsystem->ResetBackend(Backend, Error);
}
