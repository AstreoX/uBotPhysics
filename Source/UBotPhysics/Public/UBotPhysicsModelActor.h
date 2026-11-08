#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UBotPhysicsModel.h"
#include "UBotPhysicsModelActor.generated.h"

class UUBotPhysicsBindingComponent;
class UStaticMeshComponent;
class UMaterialInterface;

/** Generic presentation and binding of a model asset. Physical simulation remains entirely in its backend. */
UCLASS(BlueprintType, Blueprintable)
class UBOTPHYSICS_API AUBotPhysicsModelActor : public AActor
{
    GENERATED_BODY()
public:
    AUBotPhysicsModelActor();

    /** Immutable physics definition. Rebuild the display after selecting or reimporting an asset. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Physics")
    TObjectPtr<UUBotPhysicsModelAsset> ModelAsset;

    /** Backend chosen for this instance; an asset's required extension supplies its default. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Physics")
    FName PhysicsBackend;

    /** The common binding and state-source component, usable by attached presentation components. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TObjectPtr<UUBotPhysicsBindingComponent> PhysicsBinding;

    /** Recreates model body pivots, display geometry and stable body mappings. Refused while bound. */
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "uBot|Physics")
    bool RebuildModelDisplay();

    virtual void OnConstruction(const FTransform& Transform) override;

private:
    UPROPERTY()
    TArray<TObjectPtr<UStaticMeshComponent>> GeneratedComponents;

    UPROPERTY()
    TObjectPtr<UMaterialInterface> DisplayMaterial;

    UPROPERTY()
    TObjectPtr<UUBotPhysicsModelAsset> BuiltModelAsset;

    UPROPERTY()
    FGuid BuiltRevision;

    UPROPERTY()
    FName BuiltBackend;
};
