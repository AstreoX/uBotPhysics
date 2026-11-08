#pragma once

#include "CoreMinimal.h"
#include "ActorFactories/ActorFactory.h"
#include "UBotPhysicsModelActorFactory.generated.h"

/** Places a common physics model asset without knowing which backend produced it. */
UCLASS()
class UUBotPhysicsModelActorFactory : public UActorFactory
{
    GENERATED_BODY()
public:
    UUBotPhysicsModelActorFactory();
    virtual bool CanCreateActorFrom(const FAssetData& AssetData, FText& OutErrorMsg) override;
    virtual UObject* GetAssetFromActorInstance(AActor* ActorInstance) override;
protected:
    virtual void PostSpawnActor(UObject* Asset, AActor* NewActor) override;
};
