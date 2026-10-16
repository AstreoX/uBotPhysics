#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/CollisionProfile.h"
#include "GameFramework/Actor.h"
#include "UBotBodyState.h"
#include "UBotExtensionRegistry.h"
#include "UBotPhysics.h"
#include "UBotPhysicsBackend.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotPhysicsSubsystem.h"
#include "UBotPhysicsTestTypes.h"
#include "UBotPhysicsTestWorld.h"
#include "UBotRuntimeStatus.h"

// Backends and components run code while the subsystem is in the middle of binding or writing poses back.
// These tests make that code unbind, rebind, unregister and replace things, and check that the subsystem
// then neither writes stale poses nor leaves a half-bound binding behind.
namespace UBotPhysicsReentrancyTests
{
    using namespace UBotPhysicsTests;

    constexpr double StepSeconds = 0.01;

    /** A velocity that no solver reports, to tell "left alone" from "written". */
    const FVector Sentinel(7.0, 8.0, 9.0);

    UBoxComponent* GetBox(const AActor* Actor)
    {
        return Actor != nullptr ? Cast<UBoxComponent>(Actor->GetRootComponent()) : nullptr;
    }

    UUBotPhysicsTestBinding* AddBinding(AActor& Actor, const FVector& VelocityMetersPerSecond = FVector::ZeroVector)
    {
        UUBotPhysicsTestBinding* Binding = NewObject<UUBotPhysicsTestBinding>(&Actor);
        Binding->Backend = BackendName;
        Binding->bBindOnBeginPlay = false;
        Binding->VelocityMetersPerSecond = VelocityMetersPerSecond;
        Binding->RegisterComponent();
        return Binding;
    }

    // A box component of Actor, 50 cm cube, attached to the root at OffsetCm. With bSimulate it is a Chaos physics
    // body (which detaches it from the root, keeping its place); without, it has no collision at all.
    UBoxComponent* AddBox(AActor& Actor, const TCHAR* Name, const FVector& OffsetCm, bool bSimulate)
    {
        UBoxComponent* Box = NewObject<UBoxComponent>(&Actor, Name);
        Box->SetBoxExtent(FVector(25.0), false);
        Box->SetCollisionProfileName(bSimulate ? UCollisionProfile::PhysicsActor_ProfileName : UCollisionProfile::NoCollision_ProfileName);
        Box->SetupAttachment(Actor.GetRootComponent());
        Box->SetRelativeLocation(OffsetCm);
        Box->RegisterComponent();
        if (bSimulate)
        {
            Box->SetSimulatePhysics(true);
        }
        return Box;
    }

    FUBotPhysicsBodyBinding MakeBody(UActorComponent* Component)
    {
        FUBotPhysicsBodyBinding Body;
        Body.Component.OverrideComponent = Component;
        return Body;
    }

    FString ProblemId(const UActorComponent* Binding)
    {
        return FString::Printf(TEXT("physics.binding.%s"), *Binding->GetPathName());
    }

    FUBotPhysicsTestSolver* FindTestSolver(const UUBotPhysicsSubsystem& Subsystem)
    {
        return static_cast<FUBotPhysicsTestSolver*>(Subsystem.FindSolver(BackendName));
    }

    /** What a component's transform callback does when it fires the next time. */
    enum class ECallbackAction : uint8
    {
        None,
        UnbindOther,
        UnbindSelf,
        RebindSelf,
        RebindOther
    };

    /** Shared by value with the callback, so it outlives the test body whatever happens to the component. */
    struct FCallbackState
    {
        ECallbackAction Action = ECallbackAction::None;
        int32 Runs = 0;
        TWeakObjectPtr<UUBotPhysicsTestBinding> Self;
        TWeakObjectPtr<UUBotPhysicsTestBinding> Other;
        /** A rebinding action gives the binding it rebinds this velocity first (when set), so that the new body differs from the old one. */
        TOptional<FVector> NewVelocityMetersPerSecond;
    };

    /** Unbind or rebind from Box's transform callback, as State says; the action is used up by the first move. */
    void HookTransformCallback(UBoxComponent& Box, const TSharedRef<FCallbackState>& State)
    {
        Box.TransformUpdated.AddLambda([State](USceneComponent* /*Updated*/, EUpdateTransformFlags /*Flags*/, ETeleportType /*Teleport*/)
        {
            const ECallbackAction Action = State->Action;
            State->Action = ECallbackAction::None;
            const auto Rebind = [&State](UUBotPhysicsTestBinding* Binding)
            {
                if (Binding != nullptr)
                {
                    Binding->Unbind();
                    if (State->NewVelocityMetersPerSecond.IsSet())
                    {
                        Binding->VelocityMetersPerSecond = State->NewVelocityMetersPerSecond.GetValue();
                    }
                    Binding->Bind();
                }
            };
            switch (Action)
            {
            case ECallbackAction::UnbindOther:
                ++State->Runs;
                if (UUBotPhysicsTestBinding* Other = State->Other.Get())
                {
                    Other->Unbind();
                }
                break;
            case ECallbackAction::UnbindSelf:
                ++State->Runs;
                if (UUBotPhysicsTestBinding* Self = State->Self.Get())
                {
                    Self->Unbind();
                }
                break;
            case ECallbackAction::RebindSelf:
                ++State->Runs;
                Rebind(State->Self.Get());
                break;
            case ECallbackAction::RebindOther:
                ++State->Runs;
                Rebind(State->Other.Get());
                break;
            default:
                break;
            }
        });
    }

    void RegisterBackendAgain(const TSharedRef<FUBotPhysicsTestBackendFactory>& Factory)
    {
        FUBotExtensionRegistry::Get().RegisterExtension(UBot::Physics::BackendExtensionPoint, TestPackage, Factory);
    }

    void UnregisterBackend()
    {
        FUBotExtensionRegistry::Get().UnregisterAllFromPackage(TestPackage);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingWriteBackReentrancyTest, "UBotPhysics.Stepping.WriteBackReentrancy", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingWriteBackReentrancyTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsReentrancyTests;

    FScopedTestBackend Backend(StepSeconds);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // Two bodies of the same backend on separate actors, bound A first, so A is written before B.
    AActor* ActorA = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0), false);
    AActor* ActorB = Scope.SpawnBoxActor(FVector(0.0, 1000.0, 300.0), false);
    UBoxComponent* BoxA = GetBox(ActorA);
    UBoxComponent* BoxB = GetBox(ActorB);
    UUBotPhysicsTestBinding* BindingA = AddBinding(*ActorA, FVector(1.0, 0.0, 0.0));
    UUBotPhysicsTestBinding* BindingB = AddBinding(*ActorB, FVector(0.0, 1.0, 0.0));
    if (!TestTrue(TEXT("A binds"), BindingA->Bind()) || !TestTrue(TEXT("B binds"), BindingB->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }

    // Moving A runs this: the transform callback of a component is where user code unbinds things.
    const TSharedRef<FCallbackState> State = MakeShared<FCallbackState>();
    State->Self = BindingA;
    State->Other = BindingB;
    HookTransformCallback(*BoxA, State);

    // A frame long enough to run several steps, so that every body has a new pose to write.
    const auto RunLongFrame = [&]() -> bool
    {
        const int32 StepsBefore = Solver->StepEndTimes.Num();
        Scope.Tick(0.05f);
        return TestTrue(TEXT("Precondition: the frame ran solver steps"), Solver->StepEndTimes.Num() > StepsBefore);
    };

    Scope.RunFrames(10);
    FUBotBodyState BodyStateA;
    FUBotBodyState BodyStateB;
    if (!TestTrue(TEXT("Precondition: both bodies have a state"),
        UBot::BodyState::GetNearestBodyState(BoxA, BodyStateA) && UBot::BodyState::GetNearestBodyState(BoxB, BodyStateB)))
    {
        return false;
    }

    // 1. A's transform callback unbinds B before B is written. B must get neither the pose nor the velocity.
    BoxA->ComponentVelocity = Sentinel;
    BoxB->ComponentVelocity = Sentinel;
    const FVector LocationABefore = BoxA->GetComponentLocation();
    const FVector LocationBBefore = BoxB->GetComponentLocation();
    State->Action = ECallbackAction::UnbindOther;
    if (!RunLongFrame())
    {
        return false;
    }
    TestEqual(TEXT("1: the callback ran once"), State->Runs, 1);
    TestEqual(TEXT("1: B was unbound by it"), BindingB->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    TestEqual(TEXT("1: Chaos has B"), Subsystem->FindBodyBackend(BoxB), FName(NAME_None));
    TestTrue(TEXT("1: B did not get the pose it was prepared with"), BoxB->GetComponentLocation().Equals(LocationBBefore, 1.e-6));
    TestTrue(TEXT("1: B did not get the velocity"), BoxB->ComponentVelocity.Equals(Sentinel, 1.e-9));
    TestEqual(TEXT("1: A is still bound"), BindingA->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestTrue(TEXT("1: A was written: it moved along x"), BoxA->GetComponentLocation().X > LocationABefore.X + 1.0);
    TestTrue(TEXT("1: and got its velocity (cm/s)"), BoxA->ComponentVelocity.Equals(FVector(100.0, 0.0, 0.0), 1.e-3));
    Scope.Tick(0.05f);
    TestTrue(TEXT("1: the unbound body is not written in later frames either"), BoxB->GetComponentLocation().Equals(LocationBBefore, 1.e-6));
    TestTrue(TEXT("1: nor its velocity"), BoxB->ComponentVelocity.Equals(Sentinel, 1.e-9));

    // 2. A's transform callback unbinds A itself. The pose was written while A was still bound; the velocity,
    //    which comes after the move, must not be.
    TestTrue(TEXT("2: B binds again"), BindingB->Bind());
    Scope.RunFrames(5);
    BoxA->ComponentVelocity = Sentinel;
    BoxB->ComponentVelocity = Sentinel;
    State->Action = ECallbackAction::UnbindSelf;
    if (!RunLongFrame())
    {
        return false;
    }
    TestEqual(TEXT("2: the callback ran once"), State->Runs, 2);
    TestEqual(TEXT("2: A is unbound"), BindingA->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    TestTrue(TEXT("2: A did not get the velocity"), BoxA->ComponentVelocity.Equals(Sentinel, 1.e-9));
    TestEqual(TEXT("2: B is still bound"), BindingB->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestTrue(TEXT("2: B was written after it (cm/s)"), BoxB->ComponentVelocity.Equals(FVector(0.0, 100.0, 0.0), 1.e-3));
    const FVector LocationAAfter = BoxA->GetComponentLocation();
    Scope.Tick(0.05f);
    TestTrue(TEXT("2: the unbound A is left where it is"), BoxA->GetComponentLocation().Equals(LocationAAfter, 1.e-6));
    TestTrue(TEXT("2: with its velocity"), BoxA->ComponentVelocity.Equals(Sentinel, 1.e-9));

    // 3. A's transform callback unbinds A and binds it again: the component is bound, but to a new body, so the
    //    state read for the old one is stale.
    TestTrue(TEXT("3: A binds again"), BindingA->Bind());
    Scope.RunFrames(5);
    BoxA->ComponentVelocity = Sentinel;
    State->Action = ECallbackAction::RebindSelf;
    if (!RunLongFrame())
    {
        return false;
    }
    TestEqual(TEXT("3: the callback ran once"), State->Runs, 3);
    TestEqual(TEXT("3: A is bound again"), BindingA->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestEqual(TEXT("3: by the backend"), Subsystem->FindBodyBackend(BoxA), BackendName);
    TestTrue(TEXT("3: the velocity of the old body was not written"), BoxA->ComponentVelocity.Equals(Sentinel, 1.e-9));
    Scope.Tick(0.05f);
    TestTrue(TEXT("3: the new body is written from the next frame on (cm/s)"), BoxA->ComponentVelocity.Equals(FVector(100.0, 0.0, 0.0), 1.e-3));

    BindingA->Unbind();
    BindingB->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingWriteBackGenerationTest, "UBotPhysics.Stepping.WriteBackGeneration", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingWriteBackGenerationTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsReentrancyTests;

    // The solver hands out the handle of a released body again. A body that is unbound and bound again in the
    // middle of a write-back then has the same binding, backend, handle and solver as before: only the
    // hand-over generation tells it from the old body.
    FScopedTestBackend Backend(StepSeconds);
    Backend.Factory->bReuseHandles = true;
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // Bound A first, so that A is written before B.
    AActor* ActorA = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0), false);
    AActor* ActorB = Scope.SpawnBoxActor(FVector(0.0, 1000.0, 300.0), false);
    UBoxComponent* BoxA = GetBox(ActorA);
    UBoxComponent* BoxB = GetBox(ActorB);
    UUBotPhysicsTestBinding* BindingA = AddBinding(*ActorA, FVector(1.0, 0.0, 0.0));
    UUBotPhysicsTestBinding* BindingB = AddBinding(*ActorB, FVector(0.0, 1.0, 0.0));
    if (!TestTrue(TEXT("A binds"), BindingA->Bind()) || !TestTrue(TEXT("B binds"), BindingB->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver) || !TestEqual(TEXT("Precondition: two handles were given out"), Solver->IssuedHandles.Num(), 2))
    {
        return false;
    }

    const TSharedRef<FCallbackState> State = MakeShared<FCallbackState>();
    State->Self = BindingA;
    State->Other = BindingB;
    HookTransformCallback(*BoxA, State);

    const auto RunLongFrame = [&]() -> bool
    {
        const int32 StepsBefore = Solver->StepEndTimes.Num();
        Scope.Tick(0.05f);
        return TestTrue(TEXT("Precondition: the frame ran solver steps"), Solver->StepEndTimes.Num() > StepsBefore);
    };

    Scope.RunFrames(10);
    FUBotBodyState Ignored;
    if (!TestTrue(TEXT("Precondition: both bodies have a state"),
        UBot::BodyState::GetNearestBodyState(BoxA, Ignored) && UBot::BodyState::GetNearestBodyState(BoxB, Ignored)))
    {
        return false;
    }

    // 1. Moving A runs its callback, which unbinds B and binds it again with another velocity before B is written.
    //    The solver gives the new body the handle of the old one. B must get neither the pose nor the velocity
    //    that were read for the old body.
    const int32 HandleB = Solver->IssuedHandles[1];
    BoxA->ComponentVelocity = Sentinel;
    BoxB->ComponentVelocity = Sentinel;
    const FVector LocationABefore = BoxA->GetComponentLocation();
    const FVector LocationBBefore = BoxB->GetComponentLocation();
    State->NewVelocityMetersPerSecond = FVector(0.0, 2.0, 0.0);
    State->Action = ECallbackAction::RebindOther;
    if (!RunLongFrame())
    {
        return false;
    }
    TestEqual(TEXT("1: the callback ran once"), State->Runs, 1);
    TestEqual(TEXT("1: B is bound again"), BindingB->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestEqual(TEXT("1: by the backend"), Subsystem->FindBodyBackend(BoxB), BackendName);
    if (!TestEqual(TEXT("Precondition: the new body of B was added"), Solver->IssuedHandles.Num(), 3)
        || !TestEqual(TEXT("Precondition: the solver gave it the handle of the old body"), Solver->IssuedHandles[2], HandleB))
    {
        return false;
    }
    TestTrue(TEXT("1: B did not get the pose of the old body"), BoxB->GetComponentLocation().Equals(LocationBBefore, 1.e-6));
    TestTrue(TEXT("1: nor its velocity"), BoxB->ComponentVelocity.Equals(Sentinel, 1.e-9));
    TestTrue(TEXT("1: A was written: it moved along x"), BoxA->GetComponentLocation().X > LocationABefore.X + 1.0);
    TestTrue(TEXT("1: and got its velocity (cm/s)"), BoxA->ComponentVelocity.Equals(FVector(100.0, 0.0, 0.0), 1.e-3));
    Scope.Tick(0.05f);
    TestTrue(TEXT("1: the new body of B is written from the next frame on: its velocity (cm/s)"), BoxB->ComponentVelocity.Equals(FVector(0.0, 200.0, 0.0), 1.e-3));
    TestTrue(TEXT("1: and it moved along y"), BoxB->GetComponentLocation().Y > LocationBBefore.Y + 1.0);

    // 2. A's callback unbinds A itself and binds it again with another velocity, after the pose was written. The
    //    velocity read for the old body must not be written.
    const int32 HandleA = Solver->IssuedHandles[0];
    BoxA->ComponentVelocity = Sentinel;
    State->NewVelocityMetersPerSecond = FVector(3.0, 0.0, 0.0);
    State->Action = ECallbackAction::RebindSelf;
    if (!RunLongFrame())
    {
        return false;
    }
    TestEqual(TEXT("2: the callback ran once more"), State->Runs, 2);
    TestEqual(TEXT("2: A is bound again"), BindingA->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    if (!TestEqual(TEXT("Precondition: the new body of A was added"), Solver->IssuedHandles.Num(), 4)
        || !TestEqual(TEXT("Precondition: the solver gave it the handle of the old body"), Solver->IssuedHandles[3], HandleA))
    {
        return false;
    }
    TestTrue(TEXT("2: the velocity of the old body was not written"), BoxA->ComponentVelocity.Equals(Sentinel, 1.e-9));
    TestEqual(TEXT("2: B is still bound"), BindingB->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestTrue(TEXT("2: and was written after it (cm/s)"), BoxB->ComponentVelocity.Equals(FVector(0.0, 200.0, 0.0), 1.e-3));
    Scope.Tick(0.05f);
    TestTrue(TEXT("2: the new body of A is written from the next frame on (cm/s)"), BoxA->ComponentVelocity.Equals(FVector(300.0, 0.0, 0.0), 1.e-3));

    BindingA->Unbind();
    BindingB->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingNestedSolverCreationTest, "UBotPhysics.Binding.NestedSolverCreation", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingNestedSolverCreationTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsReentrancyTests;

    FScopedTestBackend Backend(StepSeconds);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // Two actors with a box that Chaos simulates each and a binding for the same backend.
    AActor* OuterActor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    AActor* InnerActor = Scope.SpawnBoxActor(FVector(0.0, 1000.0, 300.0));
    UBoxComponent* OuterBox = GetBox(OuterActor);
    UBoxComponent* InnerBox = GetBox(InnerActor);
    UUBotPhysicsTestBinding* Outer = AddBinding(*OuterActor, FVector(1.0, 0.0, 0.0));
    UUBotPhysicsTestBinding* Inner = AddBinding(*InnerActor, FVector(0.0, 1.0, 0.0));
    if (!TestTrue(TEXT("Precondition: Chaos simulates both boxes"), OuterBox->IsSimulatingPhysics() && InnerBox->IsSimulatingPhysics()))
    {
        return false;
    }

    // The backend's CreateSolver binds the other binding, once: the bind of Outer is still waiting for the solver.
    int32 NestedBinds = 0;
    bool bNestedResult = true;
    EUBotPhysicsBindingStatus StatusDuringCreation = EUBotPhysicsBindingStatus::Unbound;
    Backend.Factory->bKeepSolver = true;
    Backend.Factory->OnCreateSolver = [&]()
    {
        if (NestedBinds++ == 0)
        {
            bNestedResult = Inner->Bind();
            StatusDuringCreation = Inner->GetBindingStatus();
        }
    };

    TestTrue(TEXT("The outer Bind succeeds"), Outer->Bind());
    TestEqual(TEXT("The nested Bind ran once"), NestedBinds, 1);
    TestFalse(TEXT("The nested Bind fails"), bNestedResult);
    TestEqual(TEXT("It was Rejected when it returned"), StatusDuringCreation, EUBotPhysicsBindingStatus::Rejected);

    // Both bindings.
    TestEqual(TEXT("Outer is Bound"), Outer->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestTrue(TEXT("With no error"), Outer->GetBindingError().IsEmpty());
    TestEqual(TEXT("Inner is Rejected"), Inner->GetBindingStatus(), EUBotPhysicsBindingStatus::Rejected);
    const FString InnerError = Inner->GetBindingError().ToString();
    TestTrue(*FString::Printf(TEXT("Its error says the solver is still being created (\"%s\")"), *InnerError),
        InnerError.Contains(TEXT("still being created")) && InnerError.Contains(BackendName.ToString()));
    TestTrue(TEXT("Inner holds nothing"), Inner->GetBoundComponents().IsEmpty() && !Inner->ProvidesBodyState(InnerBox));
    TestEqual(TEXT("Outer holds its box"), Outer->GetBoundComponents().Num(), 1);
    TestTrue(TEXT("And provides it"), Outer->ProvidesBodyState(OuterBox));

    // One solver, with the body of Outer only.
    TestEqual(TEXT("The factory made one solver"), Backend.Factory->CreateCalls, 1);
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestTrue(TEXT("The subsystem holds that solver"), Solver != nullptr && Solver == Backend.Factory->KeptSolver.Get()))
    {
        return false;
    }
    TestEqual(TEXT("It was given one body"), Solver->AddCalls, 1);
    TestEqual(TEXT("It holds one body"), Solver->GetBodyCount(), 1);
    TestEqual(TEXT("It was given nothing back"), Solver->RemoveCalls, 0);
    TestTrue(TEXT("The body was kinematic when it got it"), Solver->bAllKinematicOnAdd);

    // Simulation flags and owners.
    TestFalse(TEXT("Chaos no longer simulates the outer box"), OuterBox->IsSimulatingPhysics());
    TestEqual(TEXT("The backend has it"), Subsystem->FindBodyBackend(OuterBox), BackendName);
    TestTrue(TEXT("Chaos still simulates the inner box"), InnerBox->IsSimulatingPhysics());
    TestEqual(TEXT("Chaos has it"), Subsystem->FindBodyBackend(InnerBox), FName(NAME_None));

    // Runtime problems: the nested failure is reported, the outer success is not.
    if (const FUBotRuntimeProblem* Problem = FUBotRuntimeStatus::Get().FindProblem(ProblemId(Inner)))
    {
        TestEqual(TEXT("The problem has the usual code"), Problem->Code, FString(TEXT("physicsBindingFailed")));
        TestEqual(TEXT("And the message of the error"), Problem->Message.ToString(), InnerError);
    }
    else
    {
        AddError(TEXT("The nested failure is reported as a runtime problem"));
    }
    TestTrue(TEXT("The outer binding reports nothing"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(Outer)) == nullptr);

    // The solver steps, because it holds a body, and only the outer box is written.
    Scope.RunFrames(20);
    TestTrue(TEXT("The solver stepped"), Solver->StepEndTimes.Num() > 0);
    TestTrue(TEXT("The outer box moved along x"), OuterBox->GetComponentLocation().X > 10.0);
    TestTrue(TEXT("With its velocity (cm/s)"), OuterBox->ComponentVelocity.Equals(FVector(100.0, 0.0, 0.0), 1.e-3));

    // Once CreateSolver is over, the inner binding binds to the same solver.
    TestTrue(TEXT("Inner binds now"), Inner->Bind());
    TestEqual(TEXT("Inner is Bound"), Inner->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestEqual(TEXT("The factory made no other solver"), Backend.Factory->CreateCalls, 1);
    TestEqual(TEXT("The nested hook did not run again"), NestedBinds, 1);
    TestTrue(TEXT("The subsystem still holds the solver"), Subsystem->FindSolver(BackendName) == Solver);
    TestEqual(TEXT("It was given the second body"), Solver->AddCalls, 2);
    TestEqual(TEXT("It holds two bodies"), Solver->GetBodyCount(), 2);
    TestTrue(TEXT("The problem is withdrawn"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(Inner)) == nullptr);
    TestFalse(TEXT("Chaos no longer simulates the inner box"), InnerBox->IsSimulatingPhysics());
    TestEqual(TEXT("The backend has it"), Subsystem->FindBodyBackend(InnerBox), BackendName);

    // Every body is written from its own handle.
    Scope.RunFrames(20);
    TestTrue(TEXT("The outer box has its velocity (cm/s)"), OuterBox->ComponentVelocity.Equals(FVector(100.0, 0.0, 0.0), 1.e-3));
    TestTrue(TEXT("The inner box has its own (cm/s)"), InnerBox->ComponentVelocity.Equals(FVector(0.0, 100.0, 0.0), 1.e-3));
    TestTrue(TEXT("The inner box moved along y"), InnerBox->GetComponentLocation().Y > 1000.0 + 1.0);
    FUBotBodyState OuterState;
    FUBotBodyState InnerState;
    if (TestTrue(TEXT("Both bodies have a state"), UBot::BodyState::GetNearestBodyState(OuterBox, OuterState) && UBot::BodyState::GetNearestBodyState(InnerBox, InnerState)))
    {
        TestTrue(TEXT("The outer state is the outer body's"), OuterState.LinearVelocityMetersPerSecond.Equals(FVector(1.0, 0.0, 0.0), 1.e-9));
        TestTrue(TEXT("The inner state is the inner body's"), InnerState.LinearVelocityMetersPerSecond.Equals(FVector(0.0, 1.0, 0.0), 1.e-9));
    }

    // Unbinding gives back exactly the body of each binding, and the body count follows.
    Outer->Unbind();
    TestEqual(TEXT("The solver let one body go"), Solver->GetBodyCount(), 1);
    TestTrue(TEXT("Chaos simulates the outer box again"), OuterBox->IsSimulatingPhysics());
    TestEqual(TEXT("The inner box stays with the backend"), Subsystem->FindBodyBackend(InnerBox), BackendName);
    Scope.RunFrames(5);
    TestTrue(TEXT("And is still written (cm/s)"), InnerBox->ComponentVelocity.Equals(FVector(0.0, 100.0, 0.0), 1.e-3));
    Inner->Unbind();
    TestEqual(TEXT("The solver holds nothing"), Solver->GetBodyCount(), 0);
    TestEqual(TEXT("It was given both back"), Solver->RemoveCalls, 2);
    TestTrue(TEXT("Chaos simulates the inner box again"), InnerBox->IsSimulatingPhysics());

    // The slot counts no body any more: an idle solver is not stepped.
    const int32 StepsWhenIdle = Solver->StepEndTimes.Num();
    Scope.RunFrames(10);
    TestEqual(TEXT("The idle solver is not stepped"), Solver->StepEndTimes.Num(), StepsWhenIdle);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingBackendLostWhileBindingTest, "UBotPhysics.Binding.BackendLostWhileBinding", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingBackendLostWhileBindingTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsReentrancyTests;

    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // An actor with three boxes that Chaos simulates and a binding that lists them all.
    const auto MakeActor = [&Scope](float OffsetCm, TArray<UBoxComponent*>& OutBoxes) -> UUBotPhysicsTestBinding*
    {
        AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, OffsetCm, 300.0));
        OutBoxes = { GetBox(Actor), AddBox(*Actor, TEXT("Second"), FVector(0.0, 500.0, 0.0), true), AddBox(*Actor, TEXT("Third"), FVector(0.0, 1000.0, 0.0), true) };
        UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, FVector(1.0, 0.0, 0.0));
        for (UBoxComponent* Box : OutBoxes)
        {
            Binding->Bodies.Add(MakeBody(Box));
        }
        return Binding;
    };
    // Everything the failed Bind must have put back.
    const auto ExpectNothingBound = [this, Subsystem](const TCHAR* Case, UUBotPhysicsTestBinding& Binding, const TArray<UBoxComponent*>& Boxes)
    {
        TestEqual(*FString::Printf(TEXT("%s: status"), Case), Binding.GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);
        const FString Error = Binding.GetBindingError().ToString();
        TestTrue(*FString::Printf(TEXT("%s: the error says the backend was unregistered or replaced (\"%s\")"), Case, *Error),
            Error.Contains(TEXT("unregistered or replaced")) && Error.Contains(BackendName.ToString()));
        TestTrue(*FString::Printf(TEXT("%s: nothing is bound"), Case), Binding.GetBoundComponents().IsEmpty());
        TestTrue(*FString::Printf(TEXT("%s: the failure is reported as a runtime problem"), Case), FUBotRuntimeStatus::Get().FindProblem(ProblemId(&Binding)) != nullptr);
        for (const UBoxComponent* Box : Boxes)
        {
            TestTrue(*FString::Printf(TEXT("%s: %s simulates again"), Case, *Box->GetName()), Box->IsSimulatingPhysics());
            TestEqual(*FString::Printf(TEXT("%s: Chaos has %s"), Case, *Box->GetName()), Subsystem->FindBodyBackend(Box), FName(NAME_None));
        }
        TestTrue(*FString::Printf(TEXT("%s: the lost backend left no solver"), Case), Subsystem->FindSolver(BackendName) == nullptr);
        TestTrue(*FString::Printf(TEXT("%s: the binding provides nothing"), Case), !Binding.ProvidesBodyState(Boxes[0]));
    };

    // 1. AddBody unregisters its own backend after accepting the second body of three.
    {
        FScopedTestBackend Backend(StepSeconds);
        if (!TestTrue(TEXT("1: the test backend is registered"), Backend.bRegistered))
        {
            return false;
        }
        Backend.Factory->bKeepSolver = true;
        Backend.Factory->OnBodyAdded = [](int32 Accepted)
        {
            if (Accepted == 2)
            {
                UnregisterBackend();
            }
        };

        TArray<UBoxComponent*> Boxes;
        UUBotPhysicsTestBinding* Binding = MakeActor(0.f, Boxes);
        TestFalse(TEXT("1: Bind fails"), Binding->Bind());
        ExpectNothingBound(TEXT("1"), *Binding, Boxes);
        if (const FUBotPhysicsTestSolver* Solver = Backend.Factory->KeptSolver.Get())
        {
            TestEqual(TEXT("1: the solver was asked for two bodies"), Solver->AddCalls, 2);
            TestEqual(TEXT("1: it was given both back"), Solver->RemoveCalls, 2);
            TestEqual(TEXT("1: it holds nothing"), Solver->GetBodyCount(), 0);
            TestTrue(TEXT("1: the bodies were kinematic when it got them"), Solver->bAllKinematicOnAdd);
        }
        else
        {
            AddError(TEXT("1: the backend made no solver"));
        }
        TestFalse(TEXT("1: the backend is gone"), UBot::Physics::FindBackendFactory(BackendName).IsValid());

        // The backend comes back as a new factory object: the next Bind uses it, and nothing of the first try is left.
        FScopedTestBackend Again(StepSeconds);
        if (TestTrue(TEXT("1: the backend registers again"), Again.bRegistered))
        {
            TestTrue(TEXT("1: Bind succeeds with it"), Binding->Bind());
            TestEqual(TEXT("1: bound"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
            TestEqual(TEXT("1: the new factory made one solver"), Again.Factory->CreateCalls, 1);
            TestEqual(TEXT("1: the three boxes are bound"), Binding->GetBoundComponents().Num(), 3);
            TestTrue(TEXT("1: the problem is withdrawn"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(Binding)) == nullptr);
            if (const FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem))
            {
                TestEqual(TEXT("1: the new solver holds three bodies"), Solver->GetBodyCount(), 3);
            }
            Binding->Unbind();
            for (const UBoxComponent* Box : Boxes)
            {
                TestTrue(TEXT("1: and gives them back"), Box->IsSimulatingPhysics());
            }
        }
    }

    // 2. CreateSolver unregisters its own backend before it returns.
    {
        FScopedTestBackend Backend(StepSeconds);
        if (!TestTrue(TEXT("2: the test backend is registered"), Backend.bRegistered))
        {
            return false;
        }
        Backend.Factory->bKeepSolver = true;
        Backend.Factory->OnCreateSolver = []()
        {
            UnregisterBackend();
        };

        TArray<UBoxComponent*> Boxes;
        UUBotPhysicsTestBinding* Binding = MakeActor(3000.f, Boxes);
        TestFalse(TEXT("2: Bind fails"), Binding->Bind());
        ExpectNothingBound(TEXT("2"), *Binding, Boxes);
        TestEqual(TEXT("2: the factory was asked once"), Backend.Factory->CreateCalls, 1);
        if (const FUBotPhysicsTestSolver* Solver = Backend.Factory->KeptSolver.Get())
        {
            TestEqual(TEXT("2: the solver was given no body"), Solver->AddCalls, 0);
        }
        else
        {
            AddError(TEXT("2: the backend made no solver"));
        }

        FScopedTestBackend Again(StepSeconds);
        if (TestTrue(TEXT("2: the backend registers again"), Again.bRegistered))
        {
            TestTrue(TEXT("2: Bind succeeds with it"), Binding->Bind());
            TestEqual(TEXT("2: the new factory made one solver"), Again.Factory->CreateCalls, 1);
            Binding->Unbind();
        }
    }

    // 3. AddBody replaces its backend by another factory object under the same name, after the first body.
    {
        FScopedTestBackend Backend(StepSeconds);
        if (!TestTrue(TEXT("3: the test backend is registered"), Backend.bRegistered))
        {
            return false;
        }
        Backend.Factory->bKeepSolver = true;
        const TSharedRef<FUBotPhysicsTestBackendFactory> Replacement = MakeShared<FUBotPhysicsTestBackendFactory>(StepSeconds);
        Backend.Factory->OnBodyAdded = [Replacement](int32 Accepted)
        {
            if (Accepted == 1)
            {
                UnregisterBackend();
                RegisterBackendAgain(Replacement);
            }
        };

        TArray<UBoxComponent*> Boxes;
        UUBotPhysicsTestBinding* Binding = MakeActor(6000.f, Boxes);
        TestFalse(TEXT("3: Bind fails"), Binding->Bind());
        TestEqual(TEXT("3: status"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);
        TestTrue(TEXT("3: nothing is bound"), Binding->GetBoundComponents().IsEmpty());
        for (const UBoxComponent* Box : Boxes)
        {
            TestTrue(*FString::Printf(TEXT("3: %s simulates again"), *Box->GetName()), Box->IsSimulatingPhysics());
            TestEqual(*FString::Printf(TEXT("3: Chaos has %s"), *Box->GetName()), Subsystem->FindBodyBackend(Box), FName(NAME_None));
        }
        if (const FUBotPhysicsTestSolver* Solver = Backend.Factory->KeptSolver.Get())
        {
            TestEqual(TEXT("3: the replaced solver was given its body back"), Solver->RemoveCalls, 1);
            TestEqual(TEXT("3: it holds nothing"), Solver->GetBodyCount(), 0);
        }
        TestTrue(TEXT("3: the subsystem holds no solver of the replaced factory"), Subsystem->FindSolver(BackendName) == nullptr);

        // The replacement is registered: the next Bind uses it.
        TestTrue(TEXT("3: Bind succeeds with the replacement"), Binding->Bind());
        TestEqual(TEXT("3: the replacement made one solver"), Replacement->CreateCalls, 1);
        TestEqual(TEXT("3: the old factory made no other"), Backend.Factory->CreateCalls, 1);
        TestTrue(TEXT("3: the subsystem holds the replacement's solver"), Subsystem->FindSolver(BackendName) != nullptr
            && Subsystem->FindSolver(BackendName) == Replacement->LastSolver.Pin().Get());
        Binding->Unbind();
    }

    return true;
}

#endif
