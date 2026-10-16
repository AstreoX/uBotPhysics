#include "UBotPhysicsSubsystem.h"

#include "Async/Async.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/Level.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "UBotBodyState.h"
#include "UBotClock.h"
#include "UBotExtensionRegistry.h"
#include "UBotPhysics.h"
#include "UBotPhysicsBackend.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotPhysicsStatus.h"

#define LOCTEXT_NAMESPACE "UBotPhysics"

namespace UBot::Physics::Private
{
    /** Steps outside this range cannot be scheduled: FUBotSampleScheduler clamps its rate to it. */
    constexpr double MinStepSeconds = 1.0 / FUBotSampleScheduler::MaxRateHz;
    constexpr double MaxStepSeconds = 1.0 / FUBotSampleScheduler::MinRateHz;

    /** Attach chains are short; the limit only guards against a corrupt (cyclic) chain. */
    constexpr int32 MaxAttachDepth = 1024;

    /** A body a binding asks for, resolved to its component. */
    struct FResolvedBody
    {
        UPrimitiveComponent* Component = nullptr;
        FName BodyName;
    };

    FString DescribeComponent(const UActorComponent* Component)
    {
        if (Component == nullptr)
        {
            return TEXT("None");
        }
        return FString::Printf(TEXT("%s.%s"), *GetNameSafe(Component->GetOwner()), *Component->GetName());
    }

    FText ComponentText(const UActorComponent* Component)
    {
        return FText::FromString(DescribeComponent(Component));
    }

    FText MakeBackendMissingText(FName Backend)
    {
        if (Backend.IsNone())
        {
            return LOCTEXT("BackendNotSet", "The binding has no backend: set Backend to the name of a registered physics backend.");
        }

        TArray<FString> Names;
        for (const FName Name : UBot::Physics::GetBackendNames())
        {
            Names.Add(Name.ToString());
        }
        return FText::FormatNamed(LOCTEXT("BackendNotRegistered", "No physics backend named '{Backend}' is registered (registered: {Registered})."),
            TEXT("Backend"), FText::FromName(Backend),
            TEXT("Registered"), Names.IsEmpty() ? LOCTEXT("NoBackends", "none") : FText::FromString(FString::Join(Names, TEXT(", "))));
    }

    FText MakeBackendUnregisteredText(FName Backend)
    {
        return FText::FormatNamed(LOCTEXT("BackendUnregistered", "Physics backend '{Backend}' was unregistered while its bodies were bound: they are back with Chaos. Bind again once the backend is registered."),
            TEXT("Backend"), FText::FromName(Backend));
    }

    FText MakeBackendLostWhileBindingText(FName Backend)
    {
        return FText::FormatNamed(LOCTEXT("BackendLostWhileBinding", "Physics backend '{Backend}' was unregistered or replaced while the bodies were handed over: nothing is bound and the components are back with Chaos. Bind again once the backend is registered."),
            TEXT("Backend"), FText::FromName(Backend));
    }

    FText MakeSolverBeingCreatedText(FName Backend)
    {
        return FText::FormatNamed(LOCTEXT("SolverBeingCreated", "The solver of physics backend '{Backend}' is still being created, so no bodies can be handed over to it yet. A backend must not bind bodies from inside CreateSolver; bind again after it returned."),
            TEXT("Backend"), FText::FromName(Backend));
    }

    /** Number of attach parents above Component. */
    int32 GetAttachDepth(const USceneComponent* Component)
    {
        int32 Depth = 0;
        for (const USceneComponent* Parent = Component->GetAttachParent(); Parent != nullptr && Depth < MaxAttachDepth; Parent = Parent->GetAttachParent())
        {
            ++Depth;
        }
        return Depth;
    }

    /** Looks up the components the binding hands over. Returns false with OutError set if one cannot be a body. */
    bool ResolveBodies(const UUBotPhysicsBindingComponent& Binding, TArray<FResolvedBody>& OutBodies, FText& OutError)
    {
        AActor* Owner = Binding.GetOwner();

        if (Binding.Bodies.IsEmpty())
        {
            USceneComponent* Root = Owner != nullptr ? Owner->GetRootComponent() : nullptr;
            if (Root == nullptr)
            {
                OutError = FText::FormatNamed(LOCTEXT("NoRoot", "Bodies is empty and {Actor} has no root component to use as the body."),
                    TEXT("Actor"), FText::FromString(GetNameSafe(Owner)));
                return false;
            }
            UPrimitiveComponent* Primitive = Cast<UPrimitiveComponent>(Root);
            if (Primitive == nullptr)
            {
                OutError = FText::FormatNamed(LOCTEXT("RootNotPrimitive", "Bodies is empty and the root component {Component} is not a primitive component, so it cannot be a physics body."),
                    TEXT("Component"), ComponentText(Root));
                return false;
            }
            FResolvedBody& Body = OutBodies.AddDefaulted_GetRef();
            Body.Component = Primitive;
            return true;
        }

        for (int32 Index = 0; Index < Binding.Bodies.Num(); ++Index)
        {
            const FUBotPhysicsBodyBinding& Entry = Binding.Bodies[Index];
            UActorComponent* Resolved = Entry.Component.GetComponent(Owner);
            if (!IsValid(Resolved))
            {
                OutError = FText::FormatNamed(LOCTEXT("BodyUnresolved", "Bodies[{Index}] does not refer to a component of {Actor}."),
                    TEXT("Index"), FText::AsNumber(Index),
                    TEXT("Actor"), FText::FromString(GetNameSafe(Owner)));
                return false;
            }
            UPrimitiveComponent* Primitive = Cast<UPrimitiveComponent>(Resolved);
            if (Primitive == nullptr)
            {
                OutError = FText::FormatNamed(LOCTEXT("BodyNotPrimitive", "Bodies[{Index}] ({Component}) is not a primitive component, so it cannot be a physics body."),
                    TEXT("Index"), FText::AsNumber(Index),
                    TEXT("Component"), ComponentText(Resolved));
                return false;
            }
            FResolvedBody& Body = OutBodies.AddDefaulted_GetRef();
            Body.Component = Primitive;
            Body.BodyName = Entry.BodyName;
        }
        return true;
    }
}

void FUBotPhysicsTickFunction::ExecuteTick(float DeltaTime, ELevelTick TickType, ENamedThreads::Type CurrentThread, const FGraphEventRef& MyCompletionGraphEvent)
{
    if (!IsValid(Target))
    {
        return;
    }
    ++ExecutedCount;
    Target->AdvanceTo(UBot::Clock::GetSimTimeSeconds(Target->GetWorld()));
}

FUBotPhysicsTickFunction::FUBotPhysicsTickFunction()
{
    bCanEverTick = true;
    bStartWithTickEnabled = true;
    TickGroup = TG_PrePhysics;
    bTickEvenWhenPaused = false;
}

FString FUBotPhysicsTickFunction::DiagnosticMessage()
{
    return TEXT("UBotPhysicsSubsystem::AdvanceTo");
}

FName FUBotPhysicsTickFunction::DiagnosticContext(bool bDetailed)
{
    return FName(TEXT("UBotPhysicsSubsystem"));
}

UUBotPhysicsSubsystem* UUBotPhysicsSubsystem::Get(const UObject* WorldContext)
{
    if (WorldContext == nullptr || GEngine == nullptr)
    {
        return nullptr;
    }
    const UWorld* World = GEngine->GetWorldFromContextObject(WorldContext, EGetWorldErrorMode::ReturnNull);
    return World != nullptr ? World->GetSubsystem<UUBotPhysicsSubsystem>() : nullptr;
}

void UUBotPhysicsSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    TickFunction.Target = this;

    // The registry broadcasts on the thread that changed it. The handler touches nothing but the weak pointer
    // until it is on the game thread.
    // The registry serializes adding and removing handlers with its broadcasts, which may come from any thread.
    ExtensionsChangedHandle = FUBotExtensionRegistry::Get().AddExtensionsChangedHandler(
        FUBotExtensionRegistry::FOnExtensionsChanged::FDelegate::CreateLambda(
            [WeakSubsystem = TWeakObjectPtr<UUBotPhysicsSubsystem>(this)](FName PointName)
            {
                HandleExtensionsChanged(WeakSubsystem, PointName);
            }));
}

void UUBotPhysicsSubsystem::Deinitialize()
{
    FUBotExtensionRegistry::Get().RemoveExtensionsChangedHandler(ExtensionsChangedHandle);
    ExtensionsChangedHandle.Reset();

    TickFunction.UnRegisterTickFunction();
    TickFunction.Target = nullptr;

    // Give every component back to Chaos while it still exists, binding by binding so each binding
    // learns that it is unbound.
    TArray<UUBotPhysicsBindingComponent*> BoundBindings;
    for (const TPair<TObjectKey<UPrimitiveComponent>, FBodySlot>& Pair : Bodies)
    {
        if (UUBotPhysicsBindingComponent* Binding = Pair.Value.Binding.Get())
        {
            BoundBindings.AddUnique(Binding);
        }
    }
    for (UUBotPhysicsBindingComponent* Binding : BoundBindings)
    {
        UnbindBodies(*Binding);
    }

    TArray<TObjectKey<UPrimitiveComponent>> Leftovers;
    Bodies.GenerateKeyArray(Leftovers);
    for (const TObjectKey<UPrimitiveComponent>& Key : Leftovers)
    {
        ReleaseBody(Key);
    }

    Solvers.Empty();

    Super::Deinitialize();
}

void UUBotPhysicsSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
    Super::OnWorldBeginPlay(InWorld);

    RegisterTickFunction(InWorld);
}

bool UUBotPhysicsSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
    return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

bool UUBotPhysicsSubsystem::BindBodies(UUBotPhysicsBindingComponent& Binding, FText& OutError)
{
    using namespace UBot::Physics::Private;

    OutError = FText::GetEmpty();
    if (Binding.GetBindingStatus() == EUBotPhysicsBindingStatus::Bound)
    {
        return true;
    }

    const FName BackendName = Binding.Backend;
    AActor* Owner = Binding.GetOwner();
    const auto Fail = [&Binding, &OutError](EUBotPhysicsBindingStatus Status, const FText& Reason)
    {
        return FailBind(Binding, Status, Reason, OutError);
    };

    // 1. The backend.
    const TSharedPtr<IUBotPhysicsBackendFactory> Factory = UBot::Physics::FindBackendFactory(BackendName);
    if (!Factory.IsValid())
    {
        return Fail(EUBotPhysicsBindingStatus::BackendMissing, MakeBackendMissingText(BackendName));
    }
    {
        // A solver made by another factory than the registered one is from a backend that was replaced; its
        // change may not have reached the game thread yet. Its bodies go back to Chaos before anything else.
        const FSolverSlot* Existing = FindSolverSlot(BackendName);
        if (Existing != nullptr && Existing->Solver.IsValid() && Existing->Factory.Pin() != Factory)
        {
            ReleaseBackend(BackendName);
        }
    }

    // 2. The components.
    FText Error;
    TArray<FResolvedBody> Resolved;
    if (!ResolveBodies(Binding, Resolved, Error))
    {
        return Fail(EUBotPhysicsBindingStatus::Rejected, Error);
    }
    const UWorld* World = GetWorld();
    for (int32 Index = 0; Index < Resolved.Num(); ++Index)
    {
        const UPrimitiveComponent* Component = Resolved[Index].Component;
        const FText Name = ComponentText(Component);

        if (Resolved.IndexOfByPredicate([Component](const FResolvedBody& Other) { return Other.Component == Component; }) != Index)
        {
            return Fail(EUBotPhysicsBindingStatus::Rejected,
                FText::FormatNamed(LOCTEXT("BodyTwice", "{Component} is listed more than once."), TEXT("Component"), Name));
        }
        if (Component->GetWorld() != World)
        {
            return Fail(EUBotPhysicsBindingStatus::Rejected,
                FText::FormatNamed(LOCTEXT("BodyOtherWorld", "{Component} belongs to another world."), TEXT("Component"), Name));
        }
        if (Component->GetOwner() != Owner)
        {
            // The body state lookup finds a source on the owner actor of a body and on that actor's components.
            return Fail(EUBotPhysicsBindingStatus::Rejected,
                FText::FormatNamed(LOCTEXT("BodyOtherActor", "{Component} belongs to another actor: a binding can only hand over components of its own actor ({Actor})."),
                    TEXT("Component"), Name, TEXT("Actor"), FText::FromString(GetNameSafe(Owner))));
        }
        if (Component->Mobility != EComponentMobility::Movable)
        {
            return Fail(EUBotPhysicsBindingStatus::Rejected,
                FText::FormatNamed(LOCTEXT("BodyNotMovable", "{Component} is not Movable: set its Mobility to Movable."), TEXT("Component"), Name));
        }
        if (const FBodySlot* Existing = FindBodySlot(Component))
        {
            return Fail(EUBotPhysicsBindingStatus::Rejected,
                FText::FormatNamed(LOCTEXT("BodyAlreadyBound", "{Component} is already bound to physics backend '{Backend}'."),
                    TEXT("Component"), Name, TEXT("Backend"), FText::FromName(Existing->Backend)));
        }
        if (const UObject* Source = UBot::BodyState::FindSource(Component))
        {
            return Fail(EUBotPhysicsBindingStatus::Rejected,
                FText::FormatNamed(LOCTEXT("BodyHasSource", "{Component} is already moved by {Source}."),
                    TEXT("Component"), Name, TEXT("Source"), FText::FromString(GetNameSafe(Source))));
        }
    }

    // 3. The solver. The backend's code runs here and may unregister or replace its own factory.
    EUBotPhysicsBindingStatus SolverFailure = EUBotPhysicsBindingStatus::Rejected;
    const int32 SlotIndex = FindOrCreateSolverSlotIndex(BackendName, Factory, Error, SolverFailure);
    if (SlotIndex == INDEX_NONE)
    {
        return Fail(SolverFailure, Error);
    }
    const TSharedPtr<IUBotPhysicsSolver> Solver = Solvers[SlotIndex].Solver;

    // 4. Hand the bodies over one by one; the first refusal undoes the ones before it.
    struct FHandedOver
    {
        UPrimitiveComponent* Component = nullptr;
        int32 Handle = INDEX_NONE;
        bool bWasSimulatingPhysics = false;
    };
    TArray<FHandedOver> HandedOver;
    // Takes back what was accepted so far, newest first, and gives the simulation flags back. The solver is held
    // here, so it exists even when its backend was unregistered and the slot let it go.
    const auto RollBack = [&Solver, &HandedOver]()
    {
        for (int32 Index = HandedOver.Num() - 1; Index >= 0; --Index)
        {
            Solver->RemoveBody(HandedOver[Index].Handle);
            if (HandedOver[Index].bWasSimulatingPhysics)
            {
                HandedOver[Index].Component->SetSimulatePhysics(true);
            }
        }
        HandedOver.Reset();
    };
    for (const FResolvedBody& Body : Resolved)
    {
        UPrimitiveComponent* Component = Body.Component;
        const bool bWasSimulating = Component->IsSimulatingPhysics();
        if (bWasSimulating)
        {
            Component->SetSimulatePhysics(false);
        }

        FUBotPhysicsBodyDesc Desc;
        Desc.Component = Component;
        Desc.BodyName = Body.BodyName;
        Desc.Binding = &Binding;
        Desc.InitialWorldTransform = Component->GetComponentTransform();

        FText BodyError;
        const int32 Handle = Solver->AddBody(Desc, BodyError);
        if (Handle >= 0)
        {
            HandedOver.Add({ Component, Handle, bWasSimulating });
        }

        // AddBody is the backend's code too. If it unregistered or replaced the backend, the body it just accepted
        // (if it did) and the earlier ones go back, whatever else it answered.
        if (!IsBackendCurrent(BackendName, Factory, Solver))
        {
            if (Handle < 0 && bWasSimulating)
            {
                Component->SetSimulatePhysics(true);
            }
            RollBack();
            return Fail(EUBotPhysicsBindingStatus::BackendMissing, MakeBackendLostWhileBindingText(BackendName));
        }

        if (Handle < 0)
        {
            if (bWasSimulating)
            {
                Component->SetSimulatePhysics(true);
            }
            RollBack();

            const FText Name = ComponentText(Component);
            return Fail(EUBotPhysicsBindingStatus::Rejected, BodyError.IsEmpty()
                ? FText::FormatNamed(LOCTEXT("BackendRefusedBody", "Physics backend '{Backend}' did not accept {Component}."),
                    TEXT("Backend"), FText::FromName(BackendName), TEXT("Component"), Name)
                : FText::FormatNamed(LOCTEXT("BackendRefusedBodyWhy", "Physics backend '{Backend}' did not accept {Component}: {Reason}"),
                    TEXT("Backend"), FText::FromName(BackendName), TEXT("Component"), Name, TEXT("Reason"), BodyError));
        }
    }

    // Everything was accepted: record it.
    FSolverSlot& Slot = Solvers[SlotIndex];
    if (Slot.BodyCount == 0)
    {
        // The clock of an idle solver starts again with its first body.
        Slot.bClockRunning = false;
    }
    TArray<TWeakObjectPtr<UPrimitiveComponent>> BoundComponents;
    for (const FHandedOver& Item : HandedOver)
    {
        FBodySlot& Body = Bodies.Add(TObjectKey<UPrimitiveComponent>(Item.Component));
        Body.Component = Item.Component;
        Body.Backend = BackendName;
        Body.Handle = Item.Handle;
        Body.Binding = &Binding;
        Body.BindingKey = TObjectKey<UUBotPhysicsBindingComponent>(&Binding);
        // Every hand-over is a new ownership, whatever handle the solver gave: see IsBodyOwnedBy.
        Body.Generation = ++NextBodyGeneration;
        Body.bWasSimulatingPhysics = Item.bWasSimulatingPhysics;
        BoundComponents.Add(Item.Component);
    }
    Slot.BodyCount += HandedOver.Num();

    Binding.SetBound(MoveTemp(BoundComponents));
    UBot::Physics::Private::ClearBindingFailure(Binding);
    return true;
}

bool UUBotPhysicsSubsystem::FailBind(UUBotPhysicsBindingComponent& Binding, EUBotPhysicsBindingStatus Status, const FText& Error, FText& OutError)
{
    OutError = Error;
    Binding.SetFailed(Status, Error);
    UBot::Physics::Private::ReportBindingFailure(Binding, Binding.Backend, Error);
    return false;
}

void UUBotPhysicsSubsystem::UnbindBodies(UUBotPhysicsBindingComponent& Binding)
{
    const TObjectKey<UUBotPhysicsBindingComponent> BindingKey(&Binding);
    TArray<TObjectKey<UPrimitiveComponent>, TInlineAllocator<8>> Keys;
    for (const TPair<TObjectKey<UPrimitiveComponent>, FBodySlot>& Pair : Bodies)
    {
        if (Pair.Value.BindingKey == BindingKey)
        {
            Keys.Add(Pair.Key);
        }
    }
    for (const TObjectKey<UPrimitiveComponent>& Key : Keys)
    {
        ReleaseBody(Key);
    }

    Binding.SetUnbound();
    UBot::Physics::Private::ClearBindingFailure(Binding);
}

FName UUBotPhysicsSubsystem::FindBodyBackend(const UPrimitiveComponent* Component) const
{
    const FBodySlot* Body = FindBodySlot(Component);
    return Body != nullptr ? Body->Backend : NAME_None;
}

bool UUBotPhysicsSubsystem::GetBodyState(const UPrimitiveComponent* Component, FUBotBodyState& OutState) const
{
    const FBodySlot* Body = FindBodySlot(Component);
    const FSolverSlot* Slot = Body != nullptr ? FindSolverSlot(Body->Backend) : nullptr;
    if (Slot == nullptr || !Slot->Solver.IsValid() || !Slot->bHasStepped)
    {
        return false;
    }

    FUBotBodyState State;
    if (!Slot->Solver->GetBodyState(Body->Handle, State))
    {
        return false;
    }
    State.TimestampSeconds = Slot->LastStepEndSeconds;
    if (State.Source.IsNone())
    {
        State.Source = Slot->Backend;
    }
    OutState = State;
    return true;
}

void UUBotPhysicsSubsystem::AdvanceTo(double SimTimeSeconds)
{
    using namespace UBot::Physics::Private;

    if (!FMath::IsFinite(SimTimeSeconds) || Solvers.IsEmpty())
    {
        return;
    }

    ReleaseStaleBodies();

    // Solvers is indexed, not iterated: a backend may bind further bodies while it steps. Slots are never
    // removed, only emptied, so an index stays valid.
    for (int32 SlotIndex = 0; SlotIndex < Solvers.Num(); ++SlotIndex)
    {
        // Held here so that a solver cannot be destroyed in the middle of its own step.
        const TSharedPtr<IUBotPhysicsSolver> Solver = Solvers[SlotIndex].Solver;
        if (!Solver.IsValid())
        {
            continue;
        }
        if (Solvers[SlotIndex].BodyCount <= 0)
        {
            Solvers[SlotIndex].bClockRunning = false;
            continue;
        }

        const double StepSeconds = Solver->GetStepSeconds();
        if (!(FMath::IsFinite(StepSeconds) && StepSeconds >= MinStepSeconds && StepSeconds <= MaxStepSeconds))
        {
            continue;
        }

        FSolverSlot& Slot = Solvers[SlotIndex];
        if (!Slot.bClockRunning)
        {
            Slot.DroppedBefore += Slot.Scheduler.GetDroppedSampleCount();
            Slot.Scheduler.Reset(1.0 / StepSeconds, SimTimeSeconds + StepSeconds);
            Slot.bClockRunning = true;
        }

        double FirstDueSeconds = 0.0;
        const int32 StepCount = Slot.Scheduler.ConsumeDueSteps(SimTimeSeconds, MaxStepsPerFrame, &FirstDueSeconds);
        if (StepCount <= 0)
        {
            continue;
        }

        const double IntervalSeconds = Slot.Scheduler.GetIntervalSeconds();
        for (int32 StepIndex = 0; StepIndex < StepCount; ++StepIndex)
        {
            // Never later than sim time, whatever the rounding of the grid says.
            const double EndSeconds = FMath::Min(FirstDueSeconds + static_cast<double>(StepIndex) * IntervalSeconds, SimTimeSeconds);
            Solver->Step(EndSeconds);

            // The solver may have lost its backend while it stepped (the game thread unregistered it).
            FSolverSlot& SteppedSlot = Solvers[SlotIndex];
            if (SteppedSlot.Solver != Solver)
            {
                break;
            }
            SteppedSlot.LastStepEndSeconds = EndSeconds;
            SteppedSlot.bHasStepped = true;
        }
    }

    // Once per frame, after every solver stepped, also for the bodies whose solver did not step this time.
    WriteBackPoses();
}

IUBotPhysicsSolver* UUBotPhysicsSubsystem::FindSolver(FName Backend) const
{
    const FSolverSlot* Slot = FindSolverSlot(Backend);
    return Slot != nullptr ? Slot->Solver.Get() : nullptr;
}

int64 UUBotPhysicsSubsystem::GetDroppedSteps(FName Backend) const
{
    const FSolverSlot* Slot = FindSolverSlot(Backend);
    return Slot != nullptr ? Slot->DroppedBefore + Slot->Scheduler.GetDroppedSampleCount() : 0;
}

void UUBotPhysicsSubsystem::SetMaxStepsPerFrame(int32 Value)
{
    MaxStepsPerFrame = FMath::Max(Value, 1);
}

void UUBotPhysicsSubsystem::RegisterTickFunction(UWorld& InWorld)
{
    if (TickFunction.IsTickFunctionRegistered() || InWorld.PersistentLevel == nullptr)
    {
        return;
    }
    TickFunction.Target = this;
    TickFunction.RegisterTickFunction(InWorld.PersistentLevel);
}

UUBotPhysicsSubsystem::FSolverSlot* UUBotPhysicsSubsystem::FindSolverSlot(FName Backend)
{
    return Solvers.FindByPredicate([Backend](const FSolverSlot& Slot) { return Slot.Backend == Backend; });
}

const UUBotPhysicsSubsystem::FSolverSlot* UUBotPhysicsSubsystem::FindSolverSlot(FName Backend) const
{
    return Solvers.FindByPredicate([Backend](const FSolverSlot& Slot) { return Slot.Backend == Backend; });
}

const UUBotPhysicsSubsystem::FBodySlot* UUBotPhysicsSubsystem::FindBodySlot(const UPrimitiveComponent* Component) const
{
    if (Component == nullptr)
    {
        return nullptr;
    }
    // The key holds the object's serial number, so a new component at the address of a destroyed one is not found.
    return Bodies.Find(TObjectKey<UPrimitiveComponent>(Component));
}

bool UUBotPhysicsSubsystem::IsBodyOwnedBy(const FBodyIdentity& Identity) const
{
    const FBodySlot* Body = Bodies.Find(Identity.ComponentKey);
    // The generation is what separates a rebound body from the old one: a solver may hand out a released handle
    // again, and then binding, backend, handle and solver are all the same.
    if (Body == nullptr || Body->BindingKey != Identity.BindingKey || Body->Backend != Identity.Backend || Body->Handle != Identity.Handle
        || Body->Generation != Identity.Generation)
    {
        return false;
    }
    const FSolverSlot* Slot = FindSolverSlot(Identity.Backend);
    const TSharedPtr<IUBotPhysicsSolver> Solver = Identity.Solver.Pin();
    return Slot != nullptr && Solver.IsValid() && Slot->Solver == Solver;
}

bool UUBotPhysicsSubsystem::IsBackendCurrent(FName Backend, const TSharedPtr<IUBotPhysicsBackendFactory>& Factory, const TSharedPtr<IUBotPhysicsSolver>& Solver) const
{
    if (!Factory.IsValid() || UBot::Physics::FindBackendFactory(Backend) != Factory)
    {
        return false;
    }
    const FSolverSlot* Slot = FindSolverSlot(Backend);
    return Slot != nullptr && Slot->Solver.IsValid() && Slot->Solver == Solver;
}

int32 UUBotPhysicsSubsystem::FindOrCreateSolverSlotIndex(FName Backend, const TSharedPtr<IUBotPhysicsBackendFactory>& Factory, FText& OutError, EUBotPhysicsBindingStatus& OutStatus)
{
    using namespace UBot::Physics::Private;

    OutStatus = EUBotPhysicsBindingStatus::Rejected;

    int32 SlotIndex = Solvers.IndexOfByPredicate([Backend](const FSolverSlot& Slot) { return Slot.Backend == Backend; });
    if (SlotIndex == INDEX_NONE)
    {
        // The slot exists before the factory runs so that it can be marked while CreateSolver runs. Slots are
        // never removed, so the index stays valid; an empty slot is a backend that has no solver.
        SlotIndex = Solvers.AddDefaulted();
        Solvers[SlotIndex].Backend = Backend;
    }
    else if (Solvers[SlotIndex].Solver.IsValid())
    {
        return SlotIndex;
    }

    if (Solvers[SlotIndex].bCreatingSolver)
    {
        // The factory's own code asked for a bind of this backend while it creates the solver. There is no solver
        // to hand bodies to yet, and creating a second one here would let the first overwrite it.
        OutError = MakeSolverBeingCreatedText(Backend);
        return INDEX_NONE;
    }

    UWorld* World = GetWorld();
    TSharedPtr<IUBotPhysicsSolver> Solver;
    if (World != nullptr)
    {
        Solvers[SlotIndex].bCreatingSolver = true;
        Solver = Factory->CreateSolver(*World);
        Solvers[SlotIndex].bCreatingSolver = false;
    }

    // CreateSolver is the backend's code: it may have unregistered or replaced its own factory. The solver it
    // returned (if any) is dropped with this call.
    if (UBot::Physics::FindBackendFactory(Backend) != Factory)
    {
        OutStatus = EUBotPhysicsBindingStatus::BackendMissing;
        OutError = MakeBackendLostWhileBindingText(Backend);
        return INDEX_NONE;
    }
    if (!Solver.IsValid())
    {
        OutError = FText::FormatNamed(LOCTEXT("SolverNotCreated", "Physics backend '{Backend}' could not create a solver for this world."),
            TEXT("Backend"), FText::FromName(Backend));
        return INDEX_NONE;
    }

    const double StepSeconds = Solver->GetStepSeconds();
    if (!(FMath::IsFinite(StepSeconds) && StepSeconds >= MinStepSeconds && StepSeconds <= MaxStepSeconds))
    {
        OutError = FText::FormatNamed(LOCTEXT("SolverBadStep", "Physics backend '{Backend}' reported the step {Step} s; it must lie between {Min} s and {Max} s."),
            TEXT("Backend"), FText::FromName(Backend),
            TEXT("Step"), FText::AsNumber(StepSeconds),
            TEXT("Min"), FText::AsNumber(MinStepSeconds),
            TEXT("Max"), FText::AsNumber(MaxStepSeconds));
        return INDEX_NONE;
    }

    // The slot takes the new solver only while it is still empty. A solver that got there meanwhile is the one the
    // slot's bodies belong to: the new one is dropped when this call returns, and the slot's is used. An empty
    // slot holds no bodies, so the body count is left alone: it is never reset here.
    FSolverSlot& Slot = Solvers[SlotIndex];
    if (!Slot.Solver.IsValid())
    {
        Slot.Solver = Solver;
        Slot.Factory = Factory;
        Slot.bClockRunning = false;
        Slot.bHasStepped = false;
        Slot.LastStepEndSeconds = 0.0;
        Slot.bWarnedNonFinite = false;
    }
    return SlotIndex;
}

void UUBotPhysicsSubsystem::ReleaseBody(TObjectKey<UPrimitiveComponent> Key)
{
    FBodySlot Body;
    if (!Bodies.RemoveAndCopyValue(Key, Body))
    {
        return;
    }

    // A component that is no longer registered (its world is going away) has no Chaos body to give back.
    UPrimitiveComponent* Component = Body.Component.Get();
    const bool bRestore = Component != nullptr && Component->IsRegistered() && Body.bWasSimulatingPhysics;

    FUBotBodyState LastState;
    bool bHaveLastState = false;
    if (FSolverSlot* Slot = FindSolverSlot(Body.Backend))
    {
        if (Slot->Solver.IsValid())
        {
            // The velocities go back to Chaos, so read them before the body leaves the solver.
            bHaveLastState = bRestore && Slot->bHasStepped && Slot->Solver->GetBodyState(Body.Handle, LastState)
                && !LastState.LinearVelocityMetersPerSecond.ContainsNaN() && !LastState.AngularVelocityRadiansPerSecond.ContainsNaN();
            Slot->Solver->RemoveBody(Body.Handle);
        }
        Slot->BodyCount = FMath::Max(Slot->BodyCount - 1, 0);
    }

    if (bRestore)
    {
        Component->SetSimulatePhysics(true);
        if (bHaveLastState)
        {
            Component->SetPhysicsLinearVelocity(LastState.LinearVelocityMetersPerSecond * 100.0);
            Component->SetPhysicsAngularVelocityInRadians(LastState.AngularVelocityRadiansPerSecond);
        }
    }
}

void UUBotPhysicsSubsystem::ReleaseStaleBodies()
{
    TArray<TObjectKey<UPrimitiveComponent>, TInlineAllocator<8>> Stale;
    for (const TPair<TObjectKey<UPrimitiveComponent>, FBodySlot>& Pair : Bodies)
    {
        if (!Pair.Value.Component.IsValid() || !Pair.Value.Binding.IsValid())
        {
            Stale.Add(Pair.Key);
        }
    }
    for (const TObjectKey<UPrimitiveComponent>& Key : Stale)
    {
        ReleaseBody(Key);
    }
}

void UUBotPhysicsSubsystem::ReleaseBackend(FName Backend)
{
    // The bodies of the backend and the bindings that hold them. A binding hands everything to one backend.
    TArray<TObjectKey<UPrimitiveComponent>, TInlineAllocator<8>> Keys;
    TArray<TWeakObjectPtr<UUBotPhysicsBindingComponent>, TInlineAllocator<4>> Bindings;
    for (const TPair<TObjectKey<UPrimitiveComponent>, FBodySlot>& Pair : Bodies)
    {
        if (Pair.Value.Backend == Backend)
        {
            Keys.Add(Pair.Key);
            if (Pair.Value.Binding.IsValid())
            {
                Bindings.AddUnique(Pair.Value.Binding);
            }
        }
    }

    // The solver still exists, so each body can hand its last velocities to Chaos.
    for (const TObjectKey<UPrimitiveComponent>& Key : Keys)
    {
        ReleaseBody(Key);
    }

    if (FSolverSlot* Slot = FindSolverSlot(Backend))
    {
        // The slot stays: the dropped step count is kept, and the next solver of this backend reuses it.
        Slot->Solver.Reset();
        Slot->Factory.Reset();
        Slot->BodyCount = 0;
        Slot->bClockRunning = false;
        Slot->bHasStepped = false;
        Slot->LastStepEndSeconds = 0.0;
        Slot->bWarnedNonFinite = false;
    }

    const FText Error = UBot::Physics::Private::MakeBackendUnregisteredText(Backend);
    for (const TWeakObjectPtr<UUBotPhysicsBindingComponent>& WeakBinding : Bindings)
    {
        if (UUBotPhysicsBindingComponent* Binding = WeakBinding.Get())
        {
            Binding->SetFailed(EUBotPhysicsBindingStatus::BackendMissing, Error);
            UBot::Physics::Private::ReportBindingFailure(*Binding, Backend, Error);
        }
    }
}

void UUBotPhysicsSubsystem::ReleaseUnregisteredBackends()
{
    TArray<FName, TInlineAllocator<4>> Lost;
    for (const FSolverSlot& Slot : Solvers)
    {
        if (!Slot.Solver.IsValid())
        {
            continue;
        }
        const TSharedPtr<IUBotPhysicsBackendFactory> Registered = UBot::Physics::FindBackendFactory(Slot.Backend);
        if (!Registered.IsValid() || Registered != Slot.Factory.Pin())
        {
            Lost.Add(Slot.Backend);
        }
    }
    for (const FName Backend : Lost)
    {
        ReleaseBackend(Backend);
    }
}

void UUBotPhysicsSubsystem::HandleExtensionsChanged(const TWeakObjectPtr<UUBotPhysicsSubsystem>& WeakSubsystem, FName PointName)
{
    if (PointName != UBot::Physics::BackendExtensionPoint)
    {
        return;
    }

    if (!IsInGameThread())
    {
        // Solvers and components belong to the game thread. The weak pointer is only resolved there, and it
        // keeps a subsystem that is gone by then from being used.
        AsyncTask(ENamedThreads::GameThread, [WeakSubsystem]()
        {
            HandleExtensionsChanged(WeakSubsystem, UBot::Physics::BackendExtensionPoint);
        });
        return;
    }

    if (UUBotPhysicsSubsystem* Subsystem = WeakSubsystem.Get())
    {
        Subsystem->ReleaseUnregisteredBackends();
    }
}

void UUBotPhysicsSubsystem::WriteBackPoses()
{
    // One body whose state is available, with where it goes. The states are collected first because moving a
    // component runs delegates that may bind, unbind or destroy bodies; Identity says who owned the body when its
    // state was read, so a write is dropped once that is no longer so.
    struct FPendingPose
    {
        TWeakObjectPtr<UPrimitiveComponent> Component;
        FBodyIdentity Identity;
        int32 AttachDepth = 0;
        int32 SlotIndex = 0;
        int32 Handle = INDEX_NONE;
        FVector LocationCm = FVector::ZeroVector;
        FQuat Rotation = FQuat::Identity;
        FVector VelocityCmPerSecond = FVector::ZeroVector;
    };
    TArray<FPendingPose, TInlineAllocator<16>> Pending;
    TArray<int32, TInlineAllocator<4>> NonFiniteSlots;

    for (const TPair<TObjectKey<UPrimitiveComponent>, FBodySlot>& Pair : Bodies)
    {
        const FBodySlot& Body = Pair.Value;
        UPrimitiveComponent* Component = Body.Component.Get();
        const int32 SlotIndex = Solvers.IndexOfByPredicate([&Body](const FSolverSlot& Slot) { return Slot.Backend == Body.Backend; });
        if (Component == nullptr || SlotIndex == INDEX_NONE)
        {
            continue;
        }
        const FSolverSlot& Slot = Solvers[SlotIndex];
        FUBotBodyState State;
        if (!Slot.Solver.IsValid() || !Slot.bHasStepped || !Slot.Solver->GetBodyState(Body.Handle, State))
        {
            continue;
        }
        if (State.PositionMeters.ContainsNaN() || State.Orientation.ContainsNaN() || State.LinearVelocityMetersPerSecond.ContainsNaN())
        {
            NonFiniteSlots.AddUnique(SlotIndex);
            continue;
        }

        FPendingPose& Item = Pending.AddDefaulted_GetRef();
        Item.Component = Component;
        Item.Identity.ComponentKey = Pair.Key;
        Item.Identity.BindingKey = Body.BindingKey;
        Item.Identity.Backend = Body.Backend;
        Item.Identity.Handle = Body.Handle;
        Item.Identity.Generation = Body.Generation;
        Item.Identity.Solver = Slot.Solver;
        Item.AttachDepth = UBot::Physics::Private::GetAttachDepth(Component);
        Item.SlotIndex = SlotIndex;
        Item.Handle = Body.Handle;
        Item.LocationCm = State.PositionMeters * 100.0;
        Item.Rotation = State.Orientation;
        if (!Item.Rotation.IsNormalized())
        {
            Item.Rotation.Normalize();
        }
        Item.VelocityCmPerSecond = State.LinearVelocityMetersPerSecond * 100.0;
    }

    // Moving a component moves everything attached to it, so a component is written before the components
    // attached to it. Bodies at the same depth do not affect each other; (slot, handle) only makes the order
    // the same on every run.
    Pending.Sort([](const FPendingPose& A, const FPendingPose& B)
    {
        if (A.AttachDepth != B.AttachDepth)
        {
            return A.AttachDepth < B.AttachDepth;
        }
        if (A.SlotIndex != B.SlotIndex)
        {
            return A.SlotIndex < B.SlotIndex;
        }
        return A.Handle < B.Handle;
    });

    for (const FPendingPose& Item : Pending)
    {
        // An earlier write (or the delegates it ran) may have unbound, rebound or destroyed this body.
        UPrimitiveComponent* Component = Item.Component.Get();
        if (Component == nullptr || !IsBodyOwnedBy(Item.Identity))
        {
            continue;
        }
        Component->SetWorldLocationAndRotation(Item.LocationCm, Item.Rotation, false, nullptr, ETeleportType::None);

        // The move ran delegates of its own: check again, and look the component up again because it may be gone.
        UPrimitiveComponent* Moved = Item.Component.Get();
        if (Moved != nullptr && IsBodyOwnedBy(Item.Identity))
        {
            Moved->ComponentVelocity = Item.VelocityCmPerSecond;
        }
    }

    for (const int32 SlotIndex : NonFiniteSlots)
    {
        if (Solvers.IsValidIndex(SlotIndex) && !Solvers[SlotIndex].bWarnedNonFinite)
        {
            Solvers[SlotIndex].bWarnedNonFinite = true;
            UE_LOG(LogUBotPhysics, Warning, TEXT("Physics backend '%s' reported a state that is not finite; the pose of that body is not written back."), *Solvers[SlotIndex].Backend.ToString());
        }
    }
}

#undef LOCTEXT_NAMESPACE
