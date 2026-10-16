#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Async/Async.h"
#include "Async/TaskGraphInterfaces.h"
#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/CollisionProfile.h"
#include "GameFramework/Actor.h"
#include "UBotBodyState.h"
#include "UBotExtensionRegistry.h"
#include "UBotPhysics.h"
#include "UBotPhysicsBackend.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotPhysicsLibrary.h"
#include "UBotPhysicsSubsystem.h"
#include "UBotPhysicsTestTypes.h"
#include "UBotPhysicsTestWorld.h"
#include "UBotRuntimeStatus.h"

namespace UBotPhysicsBindingTests
{
    using namespace UBotPhysicsTests;

    UBoxComponent* GetBox(const AActor* Actor)
    {
        return Actor != nullptr ? Cast<UBoxComponent>(Actor->GetRootComponent()) : nullptr;
    }

    // A registered test binding on Actor; call Bind yourself unless bBindOnBeginPlay is set.
    UUBotPhysicsTestBinding* AddBinding(AActor& Actor, FName Backend = BackendName, bool bBindOnBeginPlay = false)
    {
        UUBotPhysicsTestBinding* Binding = NewObject<UUBotPhysicsTestBinding>(&Actor);
        Binding->Backend = Backend;
        Binding->bBindOnBeginPlay = bBindOnBeginPlay;
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

    FUBotPhysicsBodyBinding MakeBody(UActorComponent* Component, FName BodyName = NAME_None)
    {
        FUBotPhysicsBodyBinding Body;
        Body.Component.OverrideComponent = Component;
        Body.BodyName = BodyName;
        return Body;
    }

    // Id of the runtime problem of a binding: the path name of the binding component, not the actor's name.
    FString ProblemId(const UActorComponent* Binding)
    {
        return FString::Printf(TEXT("physics.binding.%s"), *Binding->GetPathName());
    }

    FUBotPhysicsTestSolver* FindTestSolver(const UUBotPhysicsSubsystem& Subsystem)
    {
        return static_cast<FUBotPhysicsTestSolver*>(Subsystem.FindSolver(BackendName));
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingClaimAndReleaseTest, "UBotPhysics.Binding.ClaimAndRelease", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingClaimAndReleaseTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    AActor* Actor = Scope.SpawnBoxActor(FVector(100.0, 200.0, 300.0));
    UBoxComponent* Box = GetBox(Actor);
    if (!TestTrue(TEXT("Precondition: Chaos simulates the box"), Box != nullptr && Box->IsSimulatingPhysics()))
    {
        return false;
    }
    // A second primitive component on the same actor that is not handed over.
    UBoxComponent* Child = NewObject<UBoxComponent>(Actor, TEXT("Child"));
    Child->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
    Child->SetupAttachment(Box);
    Child->RegisterComponent();

    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
    Binding->VelocityMetersPerSecond = FVector(1.0, 0.0, 0.0);
    Binding->YawRateRadiansPerSecond = 0.5;

    TestEqual(TEXT("Not bound before Bind"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    TestEqual(TEXT("Chaos has the box before Bind"), Subsystem->FindBodyBackend(Box), FName(NAME_None));
    TestFalse(TEXT("The binding provides nothing before Bind"), Binding->ProvidesBodyState(Box));
    TestTrue(TEXT("No body state source before Bind"), UBot::BodyState::FindSource(Box) == nullptr);

    // Bind: the box is handed over and becomes kinematic in Chaos.
    TestTrue(TEXT("Bind succeeds"), Binding->Bind());
    TestEqual(TEXT("Bound"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestTrue(TEXT("No error"), Binding->GetBindingError().IsEmpty());
    TestFalse(TEXT("Chaos no longer simulates the box"), Box->IsSimulatingPhysics());
    TestEqual(TEXT("The backend simulates the box"), Subsystem->FindBodyBackend(Box), BackendName);
    TestEqual(TEXT("The Blueprint library reports the backend"), UUBotPhysicsLibrary::GetBodyPhysicsBackend(Box), BackendName);
    TestEqual(TEXT("A component that was not handed over stays with Chaos"), Subsystem->FindBodyBackend(Child), FName(NAME_None));
    TestEqual(TEXT("The Blueprint library reports Chaos for it"), UUBotPhysicsLibrary::GetBodyPhysicsBackend(Child), FName(NAME_None));
    const TArray<UPrimitiveComponent*> Bound = Binding->GetBoundComponents();
    if (TestEqual(TEXT("One bound component"), Bound.Num(), 1))
    {
        TestTrue(TEXT("It is the box"), Bound[0] == Box);
    }
    TestTrue(TEXT("The binding provides the box"), Binding->ProvidesBodyState(Box));
    TestFalse(TEXT("But not the child, which is no body"), Binding->ProvidesBodyState(Child));
    TestTrue(TEXT("The body state lookup finds the binding"), UBot::BodyState::FindSource(Box) == Binding);

    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The backend's solver exists"), Solver))
    {
        return false;
    }
    TestEqual(TEXT("The solver holds one body"), Solver->GetBodyCount(), 1);
    TestTrue(TEXT("The component was kinematic when the solver got it"), Solver->bAllKinematicOnAdd);
    TestTrue(TEXT("Binding again is fine"), Binding->Bind());
    TestEqual(TEXT("And hands nothing over twice"), Solver->AddCalls, 1);

    // The backend moves the box; unbinding gives it back to Chaos with the backend's last velocities.
    Scope.RunFrames(20);
    TestTrue(TEXT("The solver stepped"), Solver->StepEndTimes.Num() > 0);
    Binding->Unbind();
    TestEqual(TEXT("Unbound"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    TestTrue(TEXT("Chaos simulates the box again"), Box->IsSimulatingPhysics());
    TestEqual(TEXT("Chaos has the box again"), Subsystem->FindBodyBackend(Box), FName(NAME_None));
    TestTrue(TEXT("Nothing is bound"), Binding->GetBoundComponents().IsEmpty());
    TestFalse(TEXT("The binding provides nothing any more"), Binding->ProvidesBodyState(Box));
    TestTrue(TEXT("No body state source any more"), UBot::BodyState::FindSource(Box) == nullptr);
    TestEqual(TEXT("The solver let the body go"), Solver->GetBodyCount(), 0);
    TestEqual(TEXT("Once"), Solver->RemoveCalls, 1);
    TestTrue(TEXT("Linear velocity went back to Chaos (cm/s)"), Box->GetPhysicsLinearVelocity().Equals(FVector(100.0, 0.0, 0.0), 1.0));
    TestTrue(TEXT("Angular velocity went back to Chaos (rad/s)"), Box->GetPhysicsAngularVelocityInRadians().Equals(FVector(0.0, 0.0, 0.5), 0.01));

    // The binding can bind again.
    TestTrue(TEXT("Binds again after Unbind"), Binding->Bind());
    TestFalse(TEXT("The box is kinematic again"), Box->IsSimulatingPhysics());
    Binding->Unbind();
    TestTrue(TEXT("And is given back again"), Box->IsSimulatingPhysics());

    // A body that was not simulated stays as it was.
    AActor* Kinematic = Scope.SpawnBoxActor(FVector(0.0, 0.0, 500.0), false);
    UUBotPhysicsTestBinding* KinematicBinding = AddBinding(*Kinematic);
    TestTrue(TEXT("A component Chaos did not simulate binds"), KinematicBinding->Bind());
    KinematicBinding->Unbind();
    TestFalse(TEXT("And is not made to simulate by Unbind"), GetBox(Kinematic)->IsSimulatingPhysics());

    // Play begins: Bind On Begin Play binds, and ending play gives the body back.
    AActor* AutoActor = Scope.SpawnBoxActor(FVector(0.0, 300.0, 300.0));
    UUBotPhysicsTestBinding* AutoBinding = AddBinding(*AutoActor, BackendName, true);
    AActor* ManualActor = Scope.SpawnBoxActor(FVector(0.0, 600.0, 300.0));
    UUBotPhysicsTestBinding* ManualBinding = AddBinding(*ManualActor, BackendName, false);
    AutoActor->DispatchBeginPlay();
    ManualActor->DispatchBeginPlay();
    TestEqual(TEXT("Bind On Begin Play binds"), AutoBinding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestFalse(TEXT("The body is kinematic"), GetBox(AutoActor)->IsSimulatingPhysics());
    TestEqual(TEXT("Without it nothing is bound"), ManualBinding->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    TestTrue(TEXT("And the body stays with Chaos"), GetBox(ManualActor)->IsSimulatingPhysics());
    AutoActor->RouteEndPlay(EEndPlayReason::Destroyed);
    TestEqual(TEXT("Ending play unbinds"), AutoBinding->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    TestTrue(TEXT("And gives the body back"), GetBox(AutoActor)->IsSimulatingPhysics());

    // Destroying the binding gives the body back too.
    AActor* DestroyedActor = Scope.SpawnBoxActor(FVector(0.0, 900.0, 300.0));
    UUBotPhysicsTestBinding* DestroyedBinding = AddBinding(*DestroyedActor);
    TestTrue(TEXT("Binds"), DestroyedBinding->Bind());
    TestFalse(TEXT("Kinematic"), GetBox(DestroyedActor)->IsSimulatingPhysics());
    DestroyedBinding->DestroyComponent();
    TestTrue(TEXT("Destroying the binding gives the body back"), GetBox(DestroyedActor)->IsSimulatingPhysics());
    TestEqual(TEXT("Chaos has it"), Subsystem->FindBodyBackend(GetBox(DestroyedActor)), FName(NAME_None));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingConflictsTest, "UBotPhysics.Binding.Conflicts", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingConflictsTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // Binds Binding, expects it to be refused with ErrorPart in the message, and leaves it unbound.
    const auto ExpectRejected = [this](const TCHAR* Case, UUBotPhysicsTestBinding& Binding, const TCHAR* ErrorPart)
    {
        TestFalse(*FString::Printf(TEXT("%s: Bind fails"), Case), Binding.Bind());
        TestEqual(*FString::Printf(TEXT("%s: status"), Case), Binding.GetBindingStatus(), EUBotPhysicsBindingStatus::Rejected);
        const FString Error = Binding.GetBindingError().ToString();
        TestTrue(*FString::Printf(TEXT("%s: the error says why (\"%s\")"), Case, *Error), Error.Contains(ErrorPart));
        TestTrue(*FString::Printf(TEXT("%s: nothing is bound"), Case), Binding.GetBoundComponents().IsEmpty());
        TestTrue(*FString::Printf(TEXT("%s: the failure is reported as a runtime problem"), Case),
            FUBotRuntimeStatus::Get().FindProblem(ProblemId(&Binding)) != nullptr);
        Binding.Unbind();
        TestEqual(*FString::Printf(TEXT("%s: Unbind resets the status"), Case), Binding.GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
        TestTrue(*FString::Printf(TEXT("%s: and withdraws the problem"), Case), FUBotRuntimeStatus::Get().FindProblem(ProblemId(&Binding)) == nullptr);
    };

    // A component of another binding.
    AActor* OwnerActor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    UBoxComponent* OwnerBox = GetBox(OwnerActor);
    UUBotPhysicsTestBinding* OwnerBinding = AddBinding(*OwnerActor);
    if (!TestTrue(TEXT("The first binding binds"), OwnerBinding->Bind()))
    {
        return false;
    }
    {
        // A second binding of the same actor: with the root as the default body, and with the box listed.
        UUBotPhysicsTestBinding* Binding = AddBinding(*OwnerActor);
        ExpectRejected(TEXT("Second binding on a bound component"), *Binding, TEXT("already bound"));
        Binding->Bodies.Add(MakeBody(OwnerBox));
        ExpectRejected(TEXT("Second binding that lists the bound component"), *Binding, TEXT("already bound"));
        TestEqual(TEXT("The first binding is not disturbed"), OwnerBinding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
        TestEqual(TEXT("And still owns the box"), Subsystem->FindBodyBackend(OwnerBox), BackendName);
    }

    // A component another body state source moves.
    {
        AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 1000.0, 300.0));
        UBoxComponent* Box = GetBox(Actor);
        UUBotPhysicsTestStateSource* Source = NewObject<UUBotPhysicsTestStateSource>(Actor);
        Source->ProvidedBody = Box;
        Source->RegisterComponent();
        UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
        ExpectRejected(TEXT("Component moved by another state source"), *Binding, TEXT("already moved by"));
        TestTrue(TEXT("The refused component keeps simulating"), Box->IsSimulatingPhysics());
        TestTrue(TEXT("And the other source keeps it"), UBot::BodyState::FindSource(Box) == Source);
    }

    // A component that is not Movable.
    {
        AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 1500.0, 300.0), false, EComponentMobility::Static);
        UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
        ExpectRejected(TEXT("Static component"), *Binding, TEXT("Movable"));
    }

    // A root that is not a primitive component, and a body list entry that is not one.
    {
        AActor* Actor = Scope.Get()->SpawnActor<AActor>();
        USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
        Actor->SetRootComponent(Root);
        Root->RegisterComponent();
        UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
        ExpectRejected(TEXT("Root that is no primitive component"), *Binding, TEXT("not a primitive component"));

        Binding->Bodies.Add(MakeBody(Root));
        ExpectRejected(TEXT("Listed component that is no primitive component"), *Binding, TEXT("not a primitive component"));
    }

    // No root at all.
    {
        AActor* Actor = Scope.Get()->SpawnActor<AActor>();
        UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
        ExpectRejected(TEXT("Actor without a root"), *Binding, TEXT("no root component"));
    }

    // A body list entry that refers to nothing, and one that is listed twice.
    {
        AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 2500.0, 300.0));
        UBoxComponent* Box = GetBox(Actor);
        UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
        FUBotPhysicsBodyBinding Missing;
        Missing.Component.ComponentProperty = TEXT("NoSuchComponentProperty");
        Binding->Bodies.Add(Missing);
        ExpectRejected(TEXT("Reference to nothing"), *Binding, TEXT("does not refer to a component"));

        Binding->Bodies.Reset();
        Binding->Bodies.Add(MakeBody(Box));
        Binding->Bodies.Add(MakeBody(Box));
        ExpectRejected(TEXT("Component listed twice"), *Binding, TEXT("more than once"));
        TestTrue(TEXT("The refused component keeps simulating"), Box->IsSimulatingPhysics());
    }

    // After all that the first binding still works and the others can bind once the conflict is gone.
    OwnerBinding->Unbind();
    TestTrue(TEXT("The box is free again"), OwnerBox->IsSimulatingPhysics());
    {
        UUBotPhysicsTestBinding* Binding = AddBinding(*OwnerActor);
        TestTrue(TEXT("Another binding can take the released component"), Binding->Bind());
        TestEqual(TEXT("It is theirs now"), Subsystem->FindBodyBackend(OwnerBox), BackendName);
        TestEqual(TEXT("The binding provides it"), Binding->GetBoundComponents().Num(), 1);
        TestTrue(TEXT("The first binding does not any more"), !OwnerBinding->ProvidesBodyState(OwnerBox));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingOwnActorOnlyTest, "UBotPhysics.Binding.OwnActorOnly", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingOwnActorOnlyTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    AActor* ActorA = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    AActor* ActorB = Scope.SpawnBoxActor(FVector(0.0, 500.0, 300.0));
    UBoxComponent* BoxA = GetBox(ActorA);
    UBoxComponent* BoxB = GetBox(ActorB);

    // A binding cannot hand over the component of another actor, whether it is the only body or one of several.
    UUBotPhysicsTestBinding* Binding = AddBinding(*ActorA);
    Binding->Bodies.Add(MakeBody(BoxB));
    TestFalse(TEXT("A component of another actor is refused"), Binding->Bind());
    TestEqual(TEXT("Status is Rejected"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Rejected);
    const FString Error = Binding->GetBindingError().ToString();
    TestTrue(TEXT("The error says the component belongs to another actor"), Error.Contains(TEXT("another actor")));
    TestTrue(TEXT("It names the component and the binding's actor"), Error.Contains(ActorB->GetName()) && Error.Contains(ActorA->GetName()));
    TestTrue(TEXT("The failure is reported"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(Binding)) != nullptr);
    TestTrue(TEXT("The refused component keeps simulating"), BoxB->IsSimulatingPhysics());
    TestEqual(TEXT("And stays with Chaos"), Subsystem->FindBodyBackend(BoxB), FName(NAME_None));

    Binding->Bodies.Reset();
    Binding->Bodies.Add(MakeBody(BoxA));
    Binding->Bodies.Add(MakeBody(BoxB));
    TestFalse(TEXT("Also behind a component of its own"), Binding->Bind());
    TestTrue(TEXT("The component of its own was not taken"), BoxA->IsSimulatingPhysics());
    TestEqual(TEXT("It stays with Chaos"), Subsystem->FindBodyBackend(BoxA), FName(NAME_None));
    TestTrue(TEXT("Nothing is bound"), Binding->GetBoundComponents().IsEmpty());
    Binding->Unbind();

    // Components of the binding's own actor work: the root, a child, and a source lookup that finds the binding.
    UBoxComponent* Child = NewObject<UBoxComponent>(ActorA, TEXT("Child"));
    Child->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
    Child->SetupAttachment(BoxA);
    Child->RegisterComponent();
    Binding->Bodies.Reset();
    Binding->Bodies.Add(MakeBody(BoxA));
    Binding->VelocityMetersPerSecond = FVector(1.0, 0.0, 0.0);
    if (!TestTrue(TEXT("The component of its own actor binds"), Binding->Bind()))
    {
        return false;
    }
    TestTrue(TEXT("The body state lookup finds the binding for the bound component"), UBot::BodyState::FindSource(BoxA) == Binding);

    // The root of another actor attached below the bound body is no body, but the body state is found from it.
    AActor* ActorC = Scope.SpawnBoxActor(FVector(0.0, 1000.0, 300.0), false);
    UBoxComponent* ChildOfOtherActor = GetBox(ActorC);
    ChildOfOtherActor->AttachToComponent(BoxA, FAttachmentTransformRules::KeepWorldTransform);
    TestTrue(TEXT("Precondition: the other actor's root hangs below the bound body"), ChildOfOtherActor->GetAttachParent() == BoxA);

    FUBotBodyState State;
    TestFalse(TEXT("No state before the first step"), UBot::BodyState::GetNearestBodyState(Child, State));
    Scope.RunFrames(10);
    const USceneComponent* Body = nullptr;
    if (TestTrue(TEXT("A child of the bound body finds the body state"), UBot::BodyState::GetNearestBodyState(Child, State, &Body)))
    {
        TestTrue(TEXT("The body is the bound component"), Body == BoxA);
        TestEqual(TEXT("The state comes from the backend"), State.Source, BackendName);
        TestTrue(TEXT("The body has moved on the x axis"), State.PositionMeters.X > 0.0 && FMath::IsNearlyEqual(State.PositionMeters.Y, 0.0, 1.e-9));
    }
    Body = nullptr;
    FUBotBodyState OtherState;
    if (TestTrue(TEXT("A child of another actor finds it too"), UBot::BodyState::GetNearestBodyState(ChildOfOtherActor, OtherState, &Body)))
    {
        TestTrue(TEXT("Its nearest body is the bound component"), Body == BoxA);
        TestTrue(TEXT("With the same state"), OtherState.PositionMeters.Equals(State.PositionMeters, 1.e-12));
    }
    Binding->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingBackendMissingTest, "UBotPhysics.Binding.BackendMissing", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingBackendMissingTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    UBoxComponent* Box = GetBox(Actor);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, TEXT("NoSuchBackend"));
    const FString Id = ProblemId(Binding);

    TestFalse(TEXT("Bind fails without the backend"), Binding->Bind());
    TestEqual(TEXT("Status is BackendMissing"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);
    TestTrue(TEXT("The error names the backend"), Binding->GetBindingError().ToString().Contains(TEXT("NoSuchBackend")));
    TestTrue(TEXT("The box stays with Chaos"), Box->IsSimulatingPhysics());
    TestTrue(TEXT("Nothing is bound"), Binding->GetBoundComponents().IsEmpty());

    // The failure is reported as a runtime problem.
    const FUBotRuntimeProblem* Problem = FUBotRuntimeStatus::Get().FindProblem(Id);
    if (TestNotNull(TEXT("A runtime problem is reported"), Problem))
    {
        TestEqual(TEXT("Package"), Problem->Package, FString(TEXT("UBotPhysics")));
        TestEqual(TEXT("Code"), Problem->Code, FString(TEXT("physicsBindingFailed")));
        TestTrue(TEXT("Severity"), Problem->Severity == EUBotRuntimeSeverity::Warning);
        TestEqual(TEXT("Message"), Problem->Message.ToString(), Binding->GetBindingError().ToString());
        TestEqual(TEXT("Two parameters"), Problem->Params.Num(), 2);
        if (Problem->Params.Num() == 2)
        {
            TestEqual(TEXT("First parameter"), Problem->Params[0].Key, FString(TEXT("actor")));
            TestEqual(TEXT("It is the actor name"), Problem->Params[0].Value, Actor->GetName());
            TestEqual(TEXT("Second parameter"), Problem->Params[1].Key, FString(TEXT("backend")));
            TestEqual(TEXT("It is the backend name"), Problem->Params[1].Value, FString(TEXT("NoSuchBackend")));
        }
    }

    // Unbind withdraws it.
    Binding->Unbind();
    TestTrue(TEXT("Unbind clears the problem"), FUBotRuntimeStatus::Get().FindProblem(Id) == nullptr);
    TestEqual(TEXT("And the status"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    TestTrue(TEXT("And the error"), Binding->GetBindingError().IsEmpty());

    // No backend set at all is the same failure.
    Binding->Backend = NAME_None;
    TestFalse(TEXT("Bind fails without a backend name"), Binding->Bind());
    TestEqual(TEXT("Status is BackendMissing"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);
    TestTrue(TEXT("The error asks for a backend"), Binding->GetBindingError().ToString().Contains(TEXT("no backend")));
    Binding->Unbind();

    // A later successful Bind withdraws the problem.
    {
        FScopedTestBackend Backend;
        Binding->Backend = TEXT("NoSuchBackend");
        TestFalse(TEXT("Bind fails again"), Binding->Bind());
        TestTrue(TEXT("The problem is back"), FUBotRuntimeStatus::Get().FindProblem(Id) != nullptr);
        Binding->Backend = BackendName;
        TestTrue(TEXT("Bind succeeds once the backend is right"), Binding->Bind());
        TestTrue(TEXT("The problem is gone"), FUBotRuntimeStatus::Get().FindProblem(Id) == nullptr);
        TestEqual(TEXT("Bound"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
        Binding->Unbind();
    }

    // Ending play withdraws it too.
    Binding->Backend = TEXT("NoSuchBackend");
    Actor->DispatchBeginPlay();
    TestFalse(TEXT("Bind fails a third time"), Binding->Bind());
    TestTrue(TEXT("The problem is reported"), FUBotRuntimeStatus::Get().FindProblem(Id) != nullptr);
    Actor->RouteEndPlay(EEndPlayReason::Destroyed);
    TestTrue(TEXT("Ending play clears the problem"), FUBotRuntimeStatus::Get().FindProblem(Id) == nullptr);
    TestEqual(TEXT("And resets the binding"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);

    FUBotRuntimeStatus::Get().ClearProblem(Id);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingRollbackTest, "UBotPhysics.Binding.Rollback", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingRollbackTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // Four bodies of one actor: two simulated, one that Chaos does not simulate, and the one the solver refuses
    // in the middle. A box that Chaos simulates is detached from its parent, so they are separate bodies.
    AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    UBoxComponent* First = GetBox(Actor);
    UBoxComponent* Kinematic = AddBox(*Actor, TEXT("Kinematic"), FVector(0.0, 500.0, 0.0), false);
    UBoxComponent* Refused = AddBox(*Actor, TEXT("Refused"), FVector(0.0, 1000.0, 0.0), true);
    UBoxComponent* Last = AddBox(*Actor, TEXT("Last"), FVector(0.0, 1500.0, 0.0), true);
    if (!TestTrue(TEXT("Precondition: Chaos simulates the simulated boxes"),
        First->IsSimulatingPhysics() && Refused->IsSimulatingPhysics() && Last->IsSimulatingPhysics()))
    {
        return false;
    }

    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
    Binding->Bodies.Add(MakeBody(First, TEXT("First")));
    Binding->Bodies.Add(MakeBody(Kinematic, TEXT("Kinematic")));
    Binding->Bodies.Add(MakeBody(Refused, RejectBodyName));
    Binding->Bodies.Add(MakeBody(Last, TEXT("Last")));

    TestFalse(TEXT("Bind fails when the solver refuses one body"), Binding->Bind());
    TestEqual(TEXT("Status is Rejected"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Rejected);
    const FString Error = Binding->GetBindingError().ToString();
    TestTrue(TEXT("The error names the backend"), Error.Contains(BackendName.ToString()));
    TestTrue(TEXT("The error names the refused component"), Error.Contains(Refused->GetName()) && Error.Contains(Actor->GetName()));
    TestTrue(TEXT("The error carries the solver's reason"), Error.Contains(TEXT("refuses bodies named Reject")));

    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver was created"), Solver))
    {
        return false;
    }
    TestEqual(TEXT("The solver was asked for the bodies before the refused one"), Solver->AddCalls, 2);
    TestEqual(TEXT("It gave them back"), Solver->RemoveCalls, 2);
    TestEqual(TEXT("It holds nothing"), Solver->GetBodyCount(), 0);
    TestTrue(TEXT("Nothing is bound"), Binding->GetBoundComponents().IsEmpty());
    TestFalse(TEXT("The binding provides nothing"), Binding->ProvidesBodyState(First));
    for (const UBoxComponent* Box : { First, Kinematic, Refused, Last })
    {
        TestEqual(*FString::Printf(TEXT("%s is back with Chaos"), *Box->GetName()), Subsystem->FindBodyBackend(Box), FName(NAME_None));
    }
    TestTrue(TEXT("The first body simulates again"), First->IsSimulatingPhysics());
    TestFalse(TEXT("The body that was not simulated is still not"), Kinematic->IsSimulatingPhysics());
    TestTrue(TEXT("The refused body simulates again"), Refused->IsSimulatingPhysics());
    TestTrue(TEXT("The body after it was never touched"), Last->IsSimulatingPhysics());
    TestTrue(TEXT("The failure is reported"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(Binding)) != nullptr);

    // No ownership is left behind: another binding can take a body of the failed one.
    {
        UUBotPhysicsTestBinding* Other = AddBinding(*Actor);
        Other->Bodies.Add(MakeBody(Last));
        TestTrue(TEXT("Another binding takes the last body"), Other->Bind());
        TestEqual(TEXT("It owns it"), Subsystem->FindBodyBackend(Last), BackendName);
        Other->Unbind();
        TestTrue(TEXT("And gives it back"), Last->IsSimulatingPhysics());
    }

    // Without the refused name the same list binds.
    Binding->Bodies[2].BodyName = NAME_None;
    TestTrue(TEXT("Bind succeeds once the solver accepts every body"), Binding->Bind());
    TestEqual(TEXT("Four bound components"), Binding->GetBoundComponents().Num(), 4);
    TestTrue(TEXT("The failure is withdrawn"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(Binding)) == nullptr);
    TestEqual(TEXT("The solver holds four bodies"), Solver->GetBodyCount(), 4);
    TestFalse(TEXT("The first body is kinematic"), First->IsSimulatingPhysics());
    TestFalse(TEXT("The refused one too"), Refused->IsSimulatingPhysics());
    Binding->Unbind();
    TestTrue(TEXT("Unbind restores the bodies that simulated"), First->IsSimulatingPhysics() && Refused->IsSimulatingPhysics() && Last->IsSimulatingPhysics());
    TestFalse(TEXT("And leaves the other as it was"), Kinematic->IsSimulatingPhysics());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingSolverCreationTest, "UBotPhysics.Binding.SolverCreation", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingSolverCreationTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    UBoxComponent* Box = GetBox(Actor);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);

    // The backend cannot run in this world.
    Backend.Factory->bFailCreate = true;
    TestFalse(TEXT("Bind fails when the backend makes no solver"), Binding->Bind());
    TestEqual(TEXT("Status is Rejected"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Rejected);
    TestTrue(TEXT("The error says so"), Binding->GetBindingError().ToString().Contains(TEXT("could not create a solver")));
    TestTrue(TEXT("The box stays with Chaos"), Box->IsSimulatingPhysics());
    TestTrue(TEXT("No solver is kept"), Subsystem->FindSolver(BackendName) == nullptr);
    TestEqual(TEXT("The factory was asked once"), Backend.Factory->CreateCalls, 1);

    // A solver with a step the clock cannot follow is refused as well.
    Backend.Factory->bFailCreate = false;
    Backend.Factory->StepSeconds = 0.0;
    TestFalse(TEXT("Bind fails when the step is zero"), Binding->Bind());
    TestTrue(TEXT("The error is about the step"), Binding->GetBindingError().ToString().Contains(TEXT("step")));
    Backend.Factory->StepSeconds = 1000.0;
    TestFalse(TEXT("Bind fails when the step is too long"), Binding->Bind());
    TestTrue(TEXT("No solver is kept"), Subsystem->FindSolver(BackendName) == nullptr);
    TestTrue(TEXT("The box stays with Chaos"), Box->IsSimulatingPhysics());

    // The solver is created once, on the first Bind that works, and kept.
    Backend.Factory->StepSeconds = 0.01;
    TestTrue(TEXT("Bind succeeds with a valid solver"), Binding->Bind());
    TestTrue(TEXT("The solver is kept"), Subsystem->FindSolver(BackendName) != nullptr);
    Binding->Unbind();
    TestTrue(TEXT("Also when the last body leaves"), Subsystem->FindSolver(BackendName) != nullptr);
    const int32 Created = Backend.Factory->CreateCalls;
    TestTrue(TEXT("Binding again reuses it"), Binding->Bind());
    TestEqual(TEXT("No second solver"), Backend.Factory->CreateCalls, Created);
    Binding->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingBackendUnregisteredTest, "UBotPhysics.Binding.BackendUnregistered", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingBackendUnregisteredTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    TUniquePtr<FScopedTestBackend> First = MakeUnique<FScopedTestBackend>();
    if (!TestTrue(TEXT("The test backend is registered"), First->bRegistered))
    {
        return false;
    }
    // Kept alive on purpose: the factory object exists after it is unregistered, and still must not count.
    const TSharedRef<FUBotPhysicsTestBackendFactory> FirstFactory = First->Factory;

    AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    UBoxComponent* Box = GetBox(Actor);
    AActor* KinematicActor = Scope.SpawnBoxActor(FVector(0.0, 500.0, 300.0), false);
    UBoxComponent* KinematicBox = GetBox(KinematicActor);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
    Binding->VelocityMetersPerSecond = FVector(1.0, 0.0, 0.0);
    Binding->YawRateRadiansPerSecond = 0.5;
    UUBotPhysicsTestBinding* KinematicBinding = AddBinding(*KinematicActor);
    KinematicBinding->VelocityMetersPerSecond = FVector(0.0, 1.0, 0.0);
    if (!TestTrue(TEXT("The first binding binds"), Binding->Bind()) || !TestTrue(TEXT("The second binding binds"), KinematicBinding->Bind()))
    {
        return false;
    }

    // Run, and drop some steps on the way, so that there is a velocity to hand back and a count to keep.
    Scope.RunFrames(20);
    Subsystem->SetMaxStepsPerFrame(3);
    Scope.Tick(0.5f);
    Subsystem->SetMaxStepsPerFrame(1000);
    const int64 Dropped = Subsystem->GetDroppedSteps(BackendName);
    TestTrue(TEXT("Precondition: some steps were dropped"), Dropped > 0);
    const TWeakPtr<IUBotPhysicsSolver> FirstSolver = FirstFactory->LastSolver;
    TestTrue(TEXT("Precondition: the solver exists"), FirstSolver.IsValid() && Subsystem->FindSolver(BackendName) == FirstSolver.Pin().Get());
    TestFalse(TEXT("Precondition: the box is kinematic"), Box->IsSimulatingPhysics());

    // The backend goes away. The bodies are given back right away, in the call that unregisters it.
    First.Reset();
    TestFalse(TEXT("The backend is no longer registered"), UBot::Physics::FindBackendFactory(BackendName).IsValid());
    TestEqual(TEXT("The binding is BackendMissing"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);
    TestEqual(TEXT("So is the other one"), KinematicBinding->GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);
    const FString Error = Binding->GetBindingError().ToString();
    TestTrue(TEXT("The error says the backend was unregistered"), Error.Contains(TEXT("unregistered")) && Error.Contains(BackendName.ToString()));
    TestTrue(TEXT("Chaos simulates the box again"), Box->IsSimulatingPhysics());
    TestTrue(TEXT("With the backend's last linear velocity (cm/s)"), Box->GetPhysicsLinearVelocity().Equals(FVector(100.0, 0.0, 0.0), 1.0));
    TestTrue(TEXT("And angular velocity (rad/s)"), Box->GetPhysicsAngularVelocityInRadians().Equals(FVector(0.0, 0.0, 0.5), 0.01));
    TestFalse(TEXT("The box that Chaos did not simulate is left as it was"), KinematicBox->IsSimulatingPhysics());
    TestEqual(TEXT("Chaos has the box"), Subsystem->FindBodyBackend(Box), FName(NAME_None));
    TestEqual(TEXT("And the other"), Subsystem->FindBodyBackend(KinematicBox), FName(NAME_None));
    TestTrue(TEXT("Nothing is bound"), Binding->GetBoundComponents().IsEmpty() && KinematicBinding->GetBoundComponents().IsEmpty());
    TestFalse(TEXT("The binding provides nothing"), Binding->ProvidesBodyState(Box));
    TestTrue(TEXT("No body state source is left"), UBot::BodyState::FindSource(Box) == nullptr);
    TestTrue(TEXT("The solver is gone"), Subsystem->FindSolver(BackendName) == nullptr);
    TestFalse(TEXT("And destroyed"), FirstSolver.IsValid());
    TestEqual(TEXT("The dropped step count stays"), Subsystem->GetDroppedSteps(BackendName), Dropped);
    if (const FUBotRuntimeProblem* Problem = FUBotRuntimeStatus::Get().FindProblem(ProblemId(Binding)))
    {
        TestEqual(TEXT("The problem has the usual code"), Problem->Code, FString(TEXT("physicsBindingFailed")));
        TestEqual(TEXT("And the message of the error"), Problem->Message.ToString(), Error);
        TestTrue(TEXT("It names the backend"), Problem->Params.ContainsByPredicate([](const TPair<FString, FString>& Param)
        {
            return Param.Key == TEXT("backend") && Param.Value == BackendName.ToString();
        }));
    }
    else
    {
        AddError(TEXT("The unregistered backend is reported as a runtime problem"));
    }
    TestTrue(TEXT("For the other binding too"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(KinematicBinding)) != nullptr);

    // Frames go on without the backend, and the box that was left behind stays where it is.
    const FVector LeftAt = KinematicBox->GetComponentLocation();
    Scope.RunFrames(10);
    TestTrue(TEXT("Nothing writes poses back any more"), KinematicBox->GetComponentLocation().Equals(LeftAt, 1.e-9));
    TestEqual(TEXT("The count of dropped steps does not move"), Subsystem->GetDroppedSteps(BackendName), Dropped);

    // Unbind clears the state of a binding that lost its backend.
    KinematicBinding->Unbind();
    TestEqual(TEXT("Unbind resets the status"), KinematicBinding->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    TestTrue(TEXT("And withdraws the problem"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(KinematicBinding)) == nullptr);

    // The backend comes back as a new factory object: the next Bind uses it and gets a new solver.
    FScopedTestBackend Second;
    if (!TestTrue(TEXT("The backend registers again"), Second.bRegistered))
    {
        return false;
    }
    TestTrue(TEXT("The binding binds again"), Binding->Bind());
    TestEqual(TEXT("Bound"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestTrue(TEXT("The problem is withdrawn"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(Binding)) == nullptr);
    TestFalse(TEXT("The box is kinematic again"), Box->IsSimulatingPhysics());
    TestEqual(TEXT("The new factory made one solver"), Second.Factory->CreateCalls, 1);
    TestEqual(TEXT("The old factory made no other"), FirstFactory->CreateCalls, 1);
    IUBotPhysicsSolver* NewSolver = Subsystem->FindSolver(BackendName);
    if (TestTrue(TEXT("The subsystem holds the new solver"), NewSolver != nullptr && NewSolver == Second.Factory->LastSolver.Pin().Get()))
    {
        Scope.RunFrames(10);
        TestTrue(TEXT("The new solver steps"), static_cast<FUBotPhysicsTestSolver*>(NewSolver)->StepEndTimes.Num() > 0);
        FUBotBodyState State;
        TestTrue(TEXT("And the body has a state"), Subsystem->GetBodyState(Box, State));
    }
    TestEqual(TEXT("The dropped step count is still there"), Subsystem->GetDroppedSteps(BackendName), Dropped);
    Binding->Unbind();
    TestTrue(TEXT("The box goes back to Chaos"), Box->IsSimulatingPhysics());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingBackendUnregisteredOffThreadTest, "UBotPhysics.Binding.BackendUnregisteredOffThread", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingBackendUnregisteredOffThreadTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // The registry is changed from another thread; the subsystem hears of it there and must leave the work
    // (components, solvers) to the game thread.
    FScopedTestBackend First;
    if (!TestTrue(TEXT("The test backend is registered"), First.bRegistered))
    {
        return false;
    }
    AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    UBoxComponent* Box = GetBox(Actor);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor);
    if (!TestTrue(TEXT("Bind succeeds"), Binding->Bind()))
    {
        return false;
    }
    Scope.RunFrames(5);

    Async(EAsyncExecution::Thread, []()
    {
        FUBotExtensionRegistry::Get().UnregisterAllFromPackage(UBotPhysicsTests::TestPackage);
    }).Wait();
    TestFalse(TEXT("The backend is unregistered"), UBot::Physics::FindBackendFactory(BackendName).IsValid());
    TestEqual(TEXT("The other thread did not touch the binding"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestFalse(TEXT("Nor the component"), Box->IsSimulatingPhysics());
    TestTrue(TEXT("Nor the solver"), Subsystem->FindSolver(BackendName) != nullptr);

    FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
    TestEqual(TEXT("The game thread then took the bodies back"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);
    TestTrue(TEXT("Chaos simulates the box again"), Box->IsSimulatingPhysics());
    TestTrue(TEXT("The solver is gone"), Subsystem->FindSolver(BackendName) == nullptr);
    TestTrue(TEXT("The problem is reported"), FUBotRuntimeStatus::Get().FindProblem(ProblemId(Binding)) != nullptr);

    // The backend is replaced by another factory object under the same name, all on the other thread. The game
    // thread binds before it has heard of it: the old solver's bodies are released first, the new factory is used.
    FScopedTestBackend Second;
    if (!TestTrue(TEXT("The backend registers again"), Second.bRegistered) || !TestTrue(TEXT("The binding binds to it"), Binding->Bind()))
    {
        return false;
    }
    Scope.RunFrames(5);
    AActor* OtherActor = Scope.SpawnBoxActor(FVector(0.0, 500.0, 300.0));
    UBoxComponent* OtherBox = GetBox(OtherActor);
    UUBotPhysicsTestBinding* OtherBinding = AddBinding(*OtherActor);

    const TSharedRef<FUBotPhysicsTestBackendFactory> Replacement = MakeShared<FUBotPhysicsTestBackendFactory>();
    Async(EAsyncExecution::Thread, [Replacement]()
    {
        FUBotExtensionRegistry& Registry = FUBotExtensionRegistry::Get();
        Registry.UnregisterAllFromPackage(UBotPhysicsTests::TestPackage);
        Registry.RegisterExtension(UBot::Physics::BackendExtensionPoint, UBotPhysicsTests::TestPackage, Replacement);
    }).Wait();
    TestEqual(TEXT("The first binding still believes in its solver"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);

    TestTrue(TEXT("A new binding binds to the replacement"), OtherBinding->Bind());
    TestEqual(TEXT("The replacement made a solver"), Replacement->CreateCalls, 1);
    TestEqual(TEXT("The binding of the replaced backend lost its bodies"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);
    TestTrue(TEXT("Its box is back with Chaos"), Box->IsSimulatingPhysics());
    TestFalse(TEXT("The new binding's box is kinematic"), OtherBox->IsSimulatingPhysics());

    // What the other thread queued for the game thread finds nothing left to do.
    FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
    TestEqual(TEXT("The new binding is still bound"), OtherBinding->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestFalse(TEXT("Its box is still kinematic"), OtherBox->IsSimulatingPhysics());
    TestEqual(TEXT("No second solver was made"), Replacement->CreateCalls, 1);
    TestEqual(TEXT("The first binding is still BackendMissing"), Binding->GetBindingStatus(), EUBotPhysicsBindingStatus::BackendMissing);

    OtherBinding->Unbind();
    Binding->Unbind();
    FUBotExtensionRegistry::Get().UnregisterAllFromPackage(UBotPhysicsTests::TestPackage);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBindingProblemIdsTest, "UBotPhysics.Binding.ProblemIds", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBindingProblemIdsTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsBindingTests;

    // Two worlds, and two bindings on one actor of the first; no backend is registered, so every Bind fails.
    FScopedPhysicsTestWorld FirstScope;
    FScopedPhysicsTestWorld SecondScope;
    if (!TestNotNull(TEXT("The first world has a physics subsystem"), FirstScope.GetSubsystem())
        || !TestNotNull(TEXT("The second world has a physics subsystem"), SecondScope.GetSubsystem()))
    {
        return false;
    }

    AActor* FirstActor = FirstScope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    AActor* SecondActor = SecondScope.SpawnBoxActor(FVector(0.0, 0.0, 300.0));
    UUBotPhysicsTestBinding* FirstA = AddBinding(*FirstActor, TEXT("NoSuchBackend"));
    UUBotPhysicsTestBinding* FirstB = AddBinding(*FirstActor, TEXT("NoSuchBackend"));
    UUBotPhysicsTestBinding* Second = AddBinding(*SecondActor, TEXT("NoSuchBackend"));
    TestFalse(TEXT("The first binding fails"), FirstA->Bind());
    TestFalse(TEXT("The second binding on the same actor fails"), FirstB->Bind());
    TestFalse(TEXT("The binding in the other world fails"), Second->Bind());

    const FString IdFirstA = ProblemId(FirstA);
    const FString IdFirstB = ProblemId(FirstB);
    const FString IdSecond = ProblemId(Second);
    TestEqual(TEXT("The id is physics.binding. and the path name of the binding"), IdFirstA, FString(TEXT("physics.binding.")) + FirstA->GetPathName());
    TestNotEqual(TEXT("Two bindings on one actor have their own id"), IdFirstA, IdFirstB);
    TestNotEqual(TEXT("Bindings in different worlds have their own id"), IdFirstA, IdSecond);
    TestNotEqual(TEXT("Also when their actors are called alike"), IdFirstB, IdSecond);

    FUBotRuntimeStatus& Status = FUBotRuntimeStatus::Get();
    const FUBotRuntimeProblem* ProblemFirstA = Status.FindProblem(IdFirstA);
    const FUBotRuntimeProblem* ProblemFirstB = Status.FindProblem(IdFirstB);
    TestTrue(TEXT("Each failure is a problem of its own"), ProblemFirstA != nullptr && ProblemFirstB != nullptr && Status.FindProblem(IdSecond) != nullptr);
    if (ProblemFirstA != nullptr && ProblemFirstA->Params.Num() == 2)
    {
        TestEqual(TEXT("The parameter is still the actor name"), ProblemFirstA->Params[0].Value, FirstActor->GetName());
        TestEqual(TEXT("And the backend"), ProblemFirstA->Params[1].Value, FString(TEXT("NoSuchBackend")));
    }

    // Clearing one leaves the others.
    FirstA->Unbind();
    TestTrue(TEXT("Unbind withdraws the problem of its binding"), Status.FindProblem(IdFirstA) == nullptr);
    TestTrue(TEXT("The other binding on the actor keeps its problem"), Status.FindProblem(IdFirstB) != nullptr);
    TestTrue(TEXT("And the other world keeps its own"), Status.FindProblem(IdSecond) != nullptr);
    FirstB->Unbind();
    TestTrue(TEXT("Now the second one is gone"), Status.FindProblem(IdFirstB) == nullptr);
    TestTrue(TEXT("The other world's problem is still there"), Status.FindProblem(IdSecond) != nullptr);
    Second->Unbind();
    TestTrue(TEXT("And that is gone too"), Status.FindProblem(IdSecond) == nullptr);
    return true;
}

#endif
