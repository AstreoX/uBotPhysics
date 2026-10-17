#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UBotBodyState.h"
#include "UBotPhysicsTypes.h"
#include "UBotPhysicsBindingComponent.generated.h"

class UPrimitiveComponent;
class USceneComponent;

/**
 * Hands components of its own actor to a physics backend: from Bind until Unbind the backend simulates
 * them instead of Chaos, is stepped on the uBot sim clock and moves the components every frame. The
 * component implements the uBotCore body state source interface for exactly the components it has bound,
 * so sensors can read their true state; that lookup finds a source on the owner actor of a body, which is
 * why a binding cannot hand over components of another actor.
 *
 * Binding is all or nothing and a body belongs to at most one binding. The bound components are made
 * kinematic in Chaos (physics simulation off, collision settings untouched): queries and Chaos bodies
 * still see them, and they push Chaos bodies, but nothing pushes them back. Unbind restores the earlier
 * simulation state. When the backend is unregistered while bodies are bound, they go back to Chaos the
 * same way and the status becomes BackendMissing; Bind hands them to the backend again once it is registered.
 */
UCLASS(ClassGroup = (uBot), Blueprintable, meta = (BlueprintSpawnableComponent, DisplayName = "uBot Physics Binding"))
class UBOTPHYSICS_API UUBotPhysicsBindingComponent : public UActorComponent, public IUBotBodyStateSource
{
    GENERATED_BODY()

public:
    UUBotPhysicsBindingComponent();

    /** Backend that simulates the bodies: the name an extension is registered under at the UBotPhysics.Backend extension point. Read when Bind is called. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Physics")
    FName Backend;

    /** The bodies to hand over; each must be a component of this actor. Empty: the owner's root component, which must be a primitive component. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<FUBotPhysicsBodyBinding> Bodies;

    /** Binds when play begins. Turn it off to call Bind yourself, for example after the backend's model is ready. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Physics")
    bool bBindOnBeginPlay = true;

    /**
     * Hands every body to the backend, all or nothing. Returns true when bound, also when it already was.
     * On failure nothing is bound; Get Binding Status and Get Binding Error say why, and the failure is
     * reported as a runtime problem until the next successful Bind, Unbind or the end of play.
     */
    UFUNCTION(BlueprintCallable, Category = "uBot|Physics", meta = (DisplayName = "Bind Physics Bodies"))
    bool Bind();

    /** Gives the bodies back to Chaos: simulation is restored where it was on, with the backend's last velocities. Clears a failed Bind. */
    UFUNCTION(BlueprintCallable, Category = "uBot|Physics", meta = (DisplayName = "Unbind Physics Bodies"))
    void Unbind();

    /** Where the binding stands. */
    UFUNCTION(BlueprintPure, Category = "uBot|Physics", meta = (DisplayName = "Get Physics Binding Status"))
    EUBotPhysicsBindingStatus GetBindingStatus() const { return Status; }

    /** Why the last Bind failed; empty otherwise. */
    UFUNCTION(BlueprintPure, Category = "uBot|Physics", meta = (DisplayName = "Get Physics Binding Error"))
    FText GetBindingError() const { return BindingError; }

    /** The components handed to the backend; empty unless bound. */
    UFUNCTION(BlueprintPure, Category = "uBot|Physics", meta = (DisplayName = "Get Bound Physics Components"))
    TArray<UPrimitiveComponent*> GetBoundComponents() const;

    //~ Begin UActorComponent interface
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void OnComponentDestroyed(bool bDestroyingHierarchy) override;
    //~ End UActorComponent interface

    //~ Begin IUBotBodyStateSource interface
    virtual bool ProvidesBodyState(const USceneComponent* Body) const override;
    virtual bool GetBodyState(const USceneComponent* Body, FUBotBodyState& OutState) const override;
    //~ End IUBotBodyStateSource interface

private:
    // The subsystem owns the hand-over and tells the binding what became of it.
    friend class UUBotPhysicsSubsystem;

    /** The bound component Body is, or nullptr. */
    UPrimitiveComponent* FindBoundComponent(const USceneComponent* Body) const;

    void SetBound(TArray<TWeakObjectPtr<UPrimitiveComponent>>&& Components);
    void SetFailed(EUBotPhysicsBindingStatus FailedStatus, const FText& Error);
    void SetUnbound();

    EUBotPhysicsBindingStatus Status = EUBotPhysicsBindingStatus::Unbound;
    FText BindingError;
    TArray<TWeakObjectPtr<UPrimitiveComponent>> BoundComponents;
};
