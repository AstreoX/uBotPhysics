#include "UBotPhysicsModelActorFactory.h"

#include "AssetRegistry/AssetData.h"
#include "UBotPhysicsModelActor.h"

UUBotPhysicsModelActorFactory::UUBotPhysicsModelActorFactory()
{
    DisplayName = NSLOCTEXT("UBotPhysics", "ModelActor", "uBot Physics Model");
    NewActorClass = AUBotPhysicsModelActor::StaticClass();
}

bool UUBotPhysicsModelActorFactory::CanCreateActorFrom(const FAssetData& AssetData, FText& OutErrorMsg)
{
    if (AssetData.IsValid() && AssetData.IsInstanceOf(UUBotPhysicsModelAsset::StaticClass())) { return true; }
    OutErrorMsg = NSLOCTEXT("UBotPhysics", "NotModelAsset", "Select a uBot physics model asset.");
    return false;
}

void UUBotPhysicsModelActorFactory::PostSpawnActor(UObject* Asset, AActor* NewActor)
{
    AUBotPhysicsModelActor* ModelActor = CastChecked<AUBotPhysicsModelActor>(NewActor);
    ModelActor->ModelAsset = CastChecked<UUBotPhysicsModelAsset>(Asset);
    ModelActor->RebuildModelDisplay();
}

UObject* UUBotPhysicsModelActorFactory::GetAssetFromActorInstance(AActor* ActorInstance)
{
    const AUBotPhysicsModelActor* ModelActor = Cast<AUBotPhysicsModelActor>(ActorInstance);
    return ModelActor != nullptr ? ModelActor->ModelAsset.Get() : nullptr;
}
