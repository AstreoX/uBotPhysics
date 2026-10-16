#include "UBotPhysicsBindingComponent.h"

#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "GameFramework/Actor.h"
#include "UBotPhysics.h"
#include "UBotPhysicsStatus.h"
#include "UBotPhysicsSubsystem.h"

#define LOCTEXT_NAMESPACE "UBotPhysics"

UUBotPhysicsBindingComponent::UUBotPhysicsBindingComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
}

bool UUBotPhysicsBindingComponent::Bind()
{
    if (Status == EUBotPhysicsBindingStatus::Bound)
    {
        return true;
    }

    UUBotPhysicsSubsystem* Subsystem = UUBotPhysicsSubsystem::Get(this);
    if (Subsystem == nullptr)
    {
        const FText Error = LOCTEXT("NoSubsystem", "uBot physics runs in game and Play In Editor worlds only; this component is not in one.");
        SetFailed(EUBotPhysicsBindingStatus::Rejected, Error);
        UBot::Physics::Private::ReportBindingFailure(*this, Backend, Error);
        return false;
    }

    FText Error;
    return Subsystem->BindBodies(*this, Error);
}

void UUBotPhysicsBindingComponent::Unbind()
{
    if (UUBotPhysicsSubsystem* Subsystem = UUBotPhysicsSubsystem::Get(this))
    {
        Subsystem->UnbindBodies(*this);
        return;
    }

    // No world to give the bodies back to: it is gone, and so is what it held.
    SetUnbound();
    UBot::Physics::Private::ClearBindingFailure(*this);
}

TArray<UPrimitiveComponent*> UUBotPhysicsBindingComponent::GetBoundComponents() const
{
    TArray<UPrimitiveComponent*> Result;
    for (const TWeakObjectPtr<UPrimitiveComponent>& Component : BoundComponents)
    {
        if (UPrimitiveComponent* Primitive = Component.Get())
        {
            Result.Add(Primitive);
        }
    }
    return Result;
}

void UUBotPhysicsBindingComponent::BeginPlay()
{
    Super::BeginPlay();

    if (bBindOnBeginPlay)
    {
        Bind();
    }
}

void UUBotPhysicsBindingComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Unbind();

    Super::EndPlay(EndPlayReason);
}

void UUBotPhysicsBindingComponent::OnComponentDestroyed(bool bDestroyingHierarchy)
{
    Unbind();

    Super::OnComponentDestroyed(bDestroyingHierarchy);
}

bool UUBotPhysicsBindingComponent::ProvidesBodyState(const USceneComponent* Body) const
{
    return FindBoundComponent(Body) != nullptr;
}

bool UUBotPhysicsBindingComponent::GetBodyState(const USceneComponent* Body, FUBotBodyState& OutState) const
{
    const UPrimitiveComponent* Bound = FindBoundComponent(Body);
    const UUBotPhysicsSubsystem* Subsystem = Bound != nullptr ? UUBotPhysicsSubsystem::Get(this) : nullptr;
    return Subsystem != nullptr && Subsystem->GetBodyState(Bound, OutState);
}

UPrimitiveComponent* UUBotPhysicsBindingComponent::FindBoundComponent(const USceneComponent* Body) const
{
    if (Status != EUBotPhysicsBindingStatus::Bound || Body == nullptr)
    {
        return nullptr;
    }
    for (const TWeakObjectPtr<UPrimitiveComponent>& Component : BoundComponents)
    {
        UPrimitiveComponent* Primitive = Component.Get();
        if (Primitive != nullptr && Primitive == Body)
        {
            return Primitive;
        }
    }
    return nullptr;
}

void UUBotPhysicsBindingComponent::SetBound(TArray<TWeakObjectPtr<UPrimitiveComponent>>&& Components)
{
    Status = EUBotPhysicsBindingStatus::Bound;
    BindingError = FText::GetEmpty();
    BoundComponents = MoveTemp(Components);
}

void UUBotPhysicsBindingComponent::SetFailed(EUBotPhysicsBindingStatus FailedStatus, const FText& Error)
{
    Status = FailedStatus;
    BindingError = Error;
    BoundComponents.Reset();
}

void UUBotPhysicsBindingComponent::SetUnbound()
{
    Status = EUBotPhysicsBindingStatus::Unbound;
    BindingError = FText::GetEmpty();
    BoundComponents.Reset();
}

#undef LOCTEXT_NAMESPACE
