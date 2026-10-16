#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/CollisionProfile.h"
#include "GameFramework/Actor.h"
#include "UBotBodyState.h"
#include "UBotPhysics.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotPhysicsLibrary.h"
#include "UBotPhysicsSubsystem.h"
#include "UBotPhysicsTestTypes.h"
#include "UBotPhysicsTestWorld.h"

#include <limits>

namespace UBotPhysicsSteppingTests
{
    using namespace UBotPhysicsTests;

    // Step of the test backend in these tests.
    constexpr double StepSeconds = 0.01;

    UBoxComponent* GetBox(const AActor* Actor)
    {
        return Actor != nullptr ? Cast<UBoxComponent>(Actor->GetRootComponent()) : nullptr;
    }

    UUBotPhysicsTestBinding* AddBinding(AActor& Actor, const FVector& VelocityMetersPerSecond = FVector::ZeroVector, double YawRateRadiansPerSecond = 0.0)
    {
        UUBotPhysicsTestBinding* Binding = NewObject<UUBotPhysicsTestBinding>(&Actor);
        Binding->Backend = BackendName;
        Binding->bBindOnBeginPlay = false;
        Binding->VelocityMetersPerSecond = VelocityMetersPerSecond;
        Binding->YawRateRadiansPerSecond = YawRateRadiansPerSecond;
        Binding->RegisterComponent();
        return Binding;
    }

    FUBotPhysicsTestSolver* FindTestSolver(const UUBotPhysicsSubsystem& Subsystem, FName Backend = BackendName)
    {
        return static_cast<FUBotPhysicsTestSolver*>(Subsystem.FindSolver(Backend));
    }

    FUBotPhysicsBodyBinding MakeBody(UActorComponent* Component)
    {
        FUBotPhysicsBodyBinding Body;
        Body.Component.OverrideComponent = Component;
        return Body;
    }

    // A child box attached to Parent at RelativeLocationCm, not simulated by Chaos.
    UBoxComponent* AddChildBox(AActor& Actor, USceneComponent& Parent, const TCHAR* Name, const FVector& RelativeLocationCm)
    {
        UBoxComponent* Child = NewObject<UBoxComponent>(&Actor, Name);
        Child->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
        Child->SetupAttachment(&Parent);
        Child->SetRelativeLocation(RelativeLocationCm);
        Child->RegisterComponent();
        return Child;
    }

    // Frame times between 4 and 45 ms that never repeat (golden ratio sequence), so frames and steps do not line up.
    // None of the sums falls within 0.5 ms of a step boundary for the first 60 frames.
    float IrregularDelta(int32 Index)
    {
        const double Phase = FMath::Frac(0.6180339887498949 * static_cast<double>(Index + 1));
        return static_cast<float>(0.004 + 0.041 * Phase);
    }

    // Number of grid points ClockStart + k * StepSeconds, k >= 1, that are not later than Now.
    int32 GridCount(double ClockStart, double Now)
    {
        return FMath::FloorToInt((Now - ClockStart) / StepSeconds);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingFollowsSimClockTest, "UBotPhysics.Stepping.FollowsSimClock", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingFollowsSimClockTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSteppingTests;

    FScopedTestBackend Backend(StepSeconds);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    AActor* Actor = Scope.SpawnBoxActor(FVector::ZeroVector, false);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("Bind succeeds"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }
    TestEqual(TEXT("No step before the first frame"), Solver->StepEndTimes.Num(), 0);
    TestEqual(TEXT("The solver steps at the backend's step"), Solver->GetStepSeconds(), StepSeconds);

    // The solver's clock starts with the first frame; its steps then lie on the grid one step apart.
    constexpr int32 Frames = 60;
    double ClockStart = 0.0;
    int32 Mismatches = 0;
    int32 LateSteps = 0;
    for (int32 Frame = 0; Frame < Frames; ++Frame)
    {
        Scope.Tick(IrregularDelta(Frame));
        const double Now = Scope.Now();
        if (Frame == 0)
        {
            ClockStart = Now;
        }

        const int32 Expected = GridCount(ClockStart, Now);
        if (Solver->StepEndTimes.Num() != Expected && Mismatches++ == 0)
        {
            AddError(FString::Printf(TEXT("Frame %d at %.6f s: %d steps run, the grid has %d"), Frame, Now, Solver->StepEndTimes.Num(), Expected));
        }
        if (!Solver->StepEndTimes.IsEmpty() && Solver->StepEndTimes.Last() > Now)
        {
            ++LateSteps;
        }
    }
    TestEqual(TEXT("After every frame the number of steps equals the number of grid points passed"), Mismatches, 0);
    TestEqual(TEXT("No step ends later than sim time"), LateSteps, 0);
    TestTrue(TEXT("Enough steps ran for the test to mean something"), Solver->StepEndTimes.Num() > 50);

    int32 OffGrid = 0;
    for (int32 Index = 0; Index < Solver->StepEndTimes.Num(); ++Index)
    {
        const double OnGrid = ClockStart + static_cast<double>(Index + 1) * StepSeconds;
        if (!FMath::IsNearlyEqual(Solver->StepEndTimes[Index], OnGrid, 1.e-9))
        {
            ++OffGrid;
        }
    }
    TestEqual(TEXT("Every step ends on the grid, the first one step after the clock started"), OffGrid, 0);
    TestEqual(TEXT("Nothing is dropped when frames are shorter than the step limit"), Subsystem->GetDroppedSteps(BackendName), int64(0));
    TestEqual(TEXT("The tick function ran every frame"), Subsystem->GetTickFunction().ExecutedCount, int64(Frames));

    // The same frame at the same sim time runs no further step.
    const int32 StepsBefore = Solver->StepEndTimes.Num();
    Subsystem->AdvanceTo(Scope.Now());
    TestEqual(TEXT("A second AdvanceTo at the same sim time adds nothing"), Solver->StepEndTimes.Num(), StepsBefore);
    Subsystem->AdvanceTo(Scope.Now() - 1.0);
    TestEqual(TEXT("Nor does an earlier time"), Solver->StepEndTimes.Num(), StepsBefore);
    Subsystem->AdvanceTo(std::numeric_limits<double>::quiet_NaN());
    TestEqual(TEXT("Nor does a time that is not a number"), Solver->StepEndTimes.Num(), StepsBefore);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingDropsBeyondMaxTest, "UBotPhysics.Stepping.DropsBeyondMax", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingDropsBeyondMaxTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSteppingTests;

    FScopedTestBackend Backend(StepSeconds);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    TestEqual(TEXT("By default a frame may run 1000 steps"), Subsystem->GetMaxStepsPerFrame(), 1000);
    Subsystem->SetMaxStepsPerFrame(0);
    TestEqual(TEXT("The limit is at least 1"), Subsystem->GetMaxStepsPerFrame(), 1);
    Subsystem->SetMaxStepsPerFrame(-7);
    TestEqual(TEXT("Also for negative values"), Subsystem->GetMaxStepsPerFrame(), 1);
    constexpr int32 MaxSteps = 5;
    Subsystem->SetMaxStepsPerFrame(MaxSteps);
    TestEqual(TEXT("The limit is set"), Subsystem->GetMaxStepsPerFrame(), MaxSteps);

    AActor* Actor = Scope.SpawnBoxActor(FVector::ZeroVector, false);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("Bind succeeds"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }

    // The first frame starts the clock. A long frame then needs more steps than the limit allows.
    Scope.Tick(0.013f);
    const double ClockStart = Scope.Now();
    TestEqual(TEXT("The first frame runs no step"), Solver->StepEndTimes.Num(), 0);

    Scope.Tick(0.305f);
    const double Now = Scope.Now();
    const int32 Due = GridCount(ClockStart, Now);
    if (!TestTrue(TEXT("Precondition: the long frame needs more steps than the limit"), Due > MaxSteps))
    {
        return false;
    }
    TestEqual(TEXT("The long frame runs the limit and no more"), Solver->StepEndTimes.Num(), MaxSteps);
    TestEqual(TEXT("The rest is counted as dropped"), Subsystem->GetDroppedSteps(BackendName), int64(Due - MaxSteps));
    TestEqual(TEXT("The Blueprint library reports the same"), UUBotPhysicsLibrary::GetDroppedPhysicsSteps(Scope.Get(), BackendName), int64(Due - MaxSteps));
    TestEqual(TEXT("Another backend dropped nothing"), Subsystem->GetDroppedSteps(TEXT("NoSuchBackend")), int64(0));
    if (Solver->StepEndTimes.Num() == MaxSteps)
    {
        // The steps that run are the earliest due ones, so solver time stays behind sim time.
        TestEqual(TEXT("The first step ends one step after the clock started"), Solver->StepEndTimes[0], ClockStart + StepSeconds, 1.e-9);
        TestEqual(TEXT("The last step is the limit-th due one"), Solver->StepEndTimes.Last(), ClockStart + MaxSteps * StepSeconds, 1.e-9);
        TestTrue(TEXT("It is not later than sim time"), Solver->StepEndTimes.Last() <= Now);
    }

    // The next ordinary frame continues from the grid after the long frame; the skipped steps are not made up.
    Scope.Tick(0.0171f);
    const int32 DueAfter = GridCount(ClockStart, Scope.Now());
    TestEqual(TEXT("The next frame runs the steps that became due in it"), Solver->StepEndTimes.Num(), MaxSteps + (DueAfter - Due));
    TestEqual(TEXT("Dropped steps stay as they were"), Subsystem->GetDroppedSteps(BackendName), int64(Due - MaxSteps));

    // With a higher limit the same kind of frame is simulated in full.
    Subsystem->SetMaxStepsPerFrame(1000);
    const int32 StepsBefore = Solver->StepEndTimes.Num();
    Scope.Tick(0.35f);
    const int32 DueEnd = GridCount(ClockStart, Scope.Now());
    TestEqual(TEXT("A long frame runs every due step under a high limit"), Solver->StepEndTimes.Num() - StepsBefore, DueEnd - DueAfter);
    TestEqual(TEXT("And drops none"), Subsystem->GetDroppedSteps(BackendName), int64(Due - MaxSteps));
    TestTrue(TEXT("Solver time never passes sim time"), Solver->StepEndTimes.Last() <= Scope.Now());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingWritesBackPosesTest, "UBotPhysics.Stepping.WritesBackPoses", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingWritesBackPosesTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSteppingTests;

    FScopedTestBackend Backend(StepSeconds);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    const FVector StartCm(100.0, -50.0, 200.0);
    const FVector VelocityMetersPerSecond(1.5, 0.0, 0.5);
    constexpr double YawRate = 0.8;
    AActor* Actor = Scope.SpawnBoxActor(StartCm, false);
    UBoxComponent* Box = GetBox(Actor);
    Box->SetWorldRotation(FRotator(0.0, 30.0, 0.0));
    const FQuat StartRotation = Box->GetComponentQuat();

    // A component attached to the body is not a body, but its nearest body is.
    UBoxComponent* Child = NewObject<UBoxComponent>(Actor, TEXT("Child"));
    Child->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
    Child->SetupAttachment(Box);
    Child->SetRelativeLocation(FVector(0.0, 0.0, 30.0));
    Child->RegisterComponent();

    // A body of the same kind that nobody binds.
    AActor* OtherActor = Scope.SpawnBoxActor(FVector(1000.0, 0.0, 0.0), false);
    UBoxComponent* OtherBox = GetBox(OtherActor);

    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, VelocityMetersPerSecond, YawRate);
    if (!TestTrue(TEXT("Bind succeeds"), Binding->Bind()))
    {
        return false;
    }

    FUBotBodyState State;
    TestFalse(TEXT("No state before the first step"), Subsystem->GetBodyState(Box, State));
    TestFalse(TEXT("The body state lookup has none either"), UBot::BodyState::GetNearestBodyState(Child, State));

    Scope.RunFrames(40);
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver) || !TestTrue(TEXT("The solver stepped"), Solver->StepEndTimes.Num() > 0))
    {
        return false;
    }

    // The component sits where the solver put the body after its last step.
    const int32 Steps = Solver->StepEndTimes.Num();
    const double Elapsed = static_cast<double>(Steps) * StepSeconds;
    const FVector ExpectedLocation = StartCm + VelocityMetersPerSecond * 100.0 * Elapsed;
    const FQuat ExpectedRotation = FQuat(FVector::UpVector, YawRate * Elapsed) * StartRotation;
    TestTrue(TEXT("The component location follows the solver"), Box->GetComponentLocation().Equals(ExpectedLocation, 1.e-3));
    TestTrue(TEXT("The component rotation follows the solver"), Box->GetComponentQuat().Equals(ExpectedRotation, 1.e-5));
    TestTrue(TEXT("The component velocity is the solver's velocity in cm/s"), Box->ComponentVelocity.Equals(VelocityMetersPerSecond * 100.0, 1.e-3));
    TestTrue(TEXT("Attached components move with it"),
        Child->GetComponentLocation().Equals(Box->GetComponentLocation() + Box->GetComponentQuat().RotateVector(FVector(0.0, 0.0, 30.0)), 1.e-3));
    TestTrue(TEXT("A component that is not bound is left alone"), OtherBox->GetComponentLocation().Equals(FVector(1000.0, 0.0, 0.0), 1.e-6));

    // The state the backend reports, stamped by the subsystem.
    if (TestTrue(TEXT("The subsystem has the body's state"), Subsystem->GetBodyState(Box, State)))
    {
        TestTrue(TEXT("Position in meters"), (State.PositionMeters * 100.0).Equals(ExpectedLocation, 1.e-3));
        TestTrue(TEXT("Orientation"), State.Orientation.Equals(ExpectedRotation, 1.e-5));
        TestTrue(TEXT("Linear velocity"), State.LinearVelocityMetersPerSecond.Equals(VelocityMetersPerSecond, 1.e-9));
        TestTrue(TEXT("Angular velocity"), State.AngularVelocityRadiansPerSecond.Equals(FVector(0.0, 0.0, YawRate), 1.e-9));
        TestEqual(TEXT("Source is the backend"), State.Source, BackendName);
        TestEqual(TEXT("The timestamp is the end of the last step"), State.TimestampSeconds, Solver->StepEndTimes.Last(), 1.e-12);
        TestTrue(TEXT("Never later than sim time"), State.TimestampSeconds <= Scope.Now());
    }
    FUBotBodyState ViaBinding;
    TestTrue(TEXT("The binding hands out the same state"), Binding->GetBodyState(Box, ViaBinding) && ViaBinding.PositionMeters.Equals(State.PositionMeters, 1.e-12));
    TestFalse(TEXT("But not for a component it did not bind"), Binding->GetBodyState(Child, ViaBinding));

    // Sensors find the state through the component they sit on.
    const USceneComponent* Body = nullptr;
    FUBotBodyState ChildState;
    if (TestTrue(TEXT("The nearest body state of the child is found"), UBot::BodyState::GetNearestBodyState(Child, ChildState, &Body)))
    {
        TestTrue(TEXT("Its body is the box"), Body == Box);
        TestTrue(TEXT("It is the box's state"), ChildState.PositionMeters.Equals(State.PositionMeters, 1.e-12));
    }
    TestFalse(TEXT("A component that is not bound has no body state"), UBot::BodyState::GetNearestBodyState(OtherBox, ChildState));

    // Giving the body back ends the write-back.
    const FVector Frozen = Box->GetComponentLocation();
    Binding->Unbind();
    TestFalse(TEXT("No state once unbound"), Subsystem->GetBodyState(Box, State));
    Scope.RunFrames(10);
    TestTrue(TEXT("The component stays where it was left"), Box->GetComponentLocation().Equals(Frozen, 1.e-9));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingTickFunctionTest, "UBotPhysics.Stepping.TickFunction", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingTickFunctionTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSteppingTests;

    FScopedTestBackend Backend(StepSeconds);
    FScopedPhysicsTestWorld Scope(false);
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // The function is registered when play begins.
    const FUBotPhysicsTickFunction& TickFunction = Subsystem->GetTickFunction();
    TestFalse(TEXT("Not registered before play begins"), TickFunction.IsTickFunctionRegistered());
    Scope.BeginPlay();
    TestTrue(TEXT("Registered when play begins"), TickFunction.IsTickFunctionRegistered());
    TestTrue(TEXT("It can tick"), TickFunction.bCanEverTick);
    TestTrue(TEXT("In the pre-physics group"), TickFunction.TickGroup == TG_PrePhysics);
    TestFalse(TEXT("But not while the game is paused"), TickFunction.bTickEvenWhenPaused);
    TestTrue(TEXT("It steps this subsystem"), TickFunction.Target == Subsystem);

    AActor* Actor = Scope.SpawnBoxActor(FVector::ZeroVector, false);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("Bind succeeds"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }

    Scope.RunFrames(10);
    TestEqual(TEXT("The function ran once per frame"), TickFunction.ExecutedCount, int64(10));
    const int32 StepsRunning = Solver->StepEndTimes.Num();
    TestTrue(TEXT("The solver stepped"), StepsRunning > 0);

    // Paused: the function does not run, sim time stands still and so does the solver.
    if (!TestTrue(TEXT("The world pauses"), Scope.SetPaused(true)))
    {
        return false;
    }
    const double PausedAt = Scope.Now();
    Scope.RunFrames(10);
    TestEqual(TEXT("The function did not run while paused"), TickFunction.ExecutedCount, int64(10));
    TestEqual(TEXT("Sim time stood still"), Scope.Now(), PausedAt);
    TestEqual(TEXT("The solver did not step"), Solver->StepEndTimes.Num(), StepsRunning);

    // Unpaused: it runs again and does not catch up on the time spent paused.
    Scope.SetPaused(false);
    Scope.Tick(FrameSeconds);
    TestEqual(TEXT("The function runs again"), TickFunction.ExecutedCount, int64(11));
    const int32 Resumed = Solver->StepEndTimes.Num() - StepsRunning;
    TestTrue(*FString::Printf(TEXT("The first frame after the pause runs one frame's worth of steps (%d)"), Resumed), Resumed >= 1 && Resumed <= 2);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingIdleSolverRestartsTest, "UBotPhysics.Stepping.IdleSolverRestarts", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingIdleSolverRestartsTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSteppingTests;

    FScopedTestBackend Backend(StepSeconds);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    AActor* Actor = Scope.SpawnBoxActor(FVector::ZeroVector, false);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("Bind succeeds"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }

    Scope.RunFrames(10);
    Binding->Unbind();
    const int32 StepsAtUnbind = Solver->StepEndTimes.Num();
    TestTrue(TEXT("The solver stepped while it had a body"), StepsAtUnbind > 0);

    // A solver without bodies is not stepped, and time passing meanwhile is no debt.
    Scope.RunFrames(60);
    TestEqual(TEXT("An idle solver runs no steps"), Solver->StepEndTimes.Num(), StepsAtUnbind);

    TestTrue(TEXT("Binds again"), Binding->Bind());
    Scope.Tick(FrameSeconds);
    TestEqual(TEXT("The clock starts again: the first frame runs no step"), Solver->StepEndTimes.Num(), StepsAtUnbind);
    Scope.RunFrames(30);
    const int32 Since = Solver->StepEndTimes.Num() - StepsAtUnbind;
    TestTrue(*FString::Printf(TEXT("Half a second later about 50 steps ran (%d), not the second that passed while idle"), Since), Since >= 49 && Since <= 51);
    TestEqual(TEXT("Nothing is counted as dropped"), Subsystem->GetDroppedSteps(BackendName), int64(0));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingWriteBackOrderTest, "UBotPhysics.Stepping.WriteBackOrder", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingWriteBackOrderTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSteppingTests;

    FScopedTestBackend Backend(StepSeconds);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // A chain of three bound bodies, handed over deepest first: moving a parent moves the children attached to
    // it, so writing a child before its parent leaves it off its solver pose.
    const FVector VelocityMetersPerSecond(1.0, 0.0, 0.0);
    constexpr double YawRate = 0.8;
    AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0), false);
    UBoxComponent* Parent = GetBox(Actor);
    Parent->SetWorldRotation(FRotator(0.0, 20.0, 0.0));
    UBoxComponent* Child = AddChildBox(*Actor, *Parent, TEXT("Child"), FVector(0.0, 100.0, 0.0));
    UBoxComponent* Grandchild = AddChildBox(*Actor, *Child, TEXT("Grandchild"), FVector(0.0, 0.0, 100.0));
    const TArray<UBoxComponent*> Chain = { Parent, Child, Grandchild };
    TArray<FTransform> Starts;
    for (const UBoxComponent* Box : Chain)
    {
        Starts.Add(Box->GetComponentTransform());
    }

    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, VelocityMetersPerSecond, YawRate);
    Binding->Bodies.Add(MakeBody(Grandchild));
    Binding->Bodies.Add(MakeBody(Child));
    Binding->Bodies.Add(MakeBody(Parent));
    if (!TestTrue(TEXT("Bind succeeds"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }

    for (int32 Round = 0; Round < 3; ++Round)
    {
        Scope.RunFrames(7);
        const int32 Steps = Solver->StepEndTimes.Num();
        if (!TestTrue(TEXT("The solver stepped"), Steps > 0))
        {
            return false;
        }
        const double Elapsed = static_cast<double>(Steps) * StepSeconds;
        for (int32 Index = 0; Index < Chain.Num(); ++Index)
        {
            const FVector ExpectedLocation = Starts[Index].GetLocation() + VelocityMetersPerSecond * 100.0 * Elapsed;
            const FQuat ExpectedRotation = FQuat(FVector::UpVector, YawRate * Elapsed) * Starts[Index].GetRotation();
            TestTrue(*FString::Printf(TEXT("Round %d: %s is at its solver position (attach depth %d)"), Round, *Chain[Index]->GetName(), Index),
                Chain[Index]->GetComponentLocation().Equals(ExpectedLocation, 1.e-3));
            TestTrue(*FString::Printf(TEXT("Round %d: %s has its solver rotation"), Round, *Chain[Index]->GetName()),
                Chain[Index]->GetComponentQuat().Equals(ExpectedRotation, 1.e-5));
        }
    }

    Binding->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingWriteBackAcrossBackendsTest, "UBotPhysics.Stepping.WriteBackAcrossBackends", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingWriteBackAcrossBackendsTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSteppingTests;

    const FName SecondBackendName(TEXT("UBotPhysicsTestSecond"));
    FScopedTestBackend First(StepSeconds);
    FScopedTestBackend Second(StepSeconds, SecondBackendName, TEXT("UBotPhysicsTestsSecond"));
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("Both test backends are registered"), First.bRegistered && Second.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // The root of one actor hangs below the body of another actor, and the two bodies belong to different
    // backends. The child's backend is bound first, so its solver comes first.
    AActor* ParentActor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0), false);
    AActor* ChildActor = Scope.SpawnBoxActor(FVector(0.0, 200.0, 300.0), false);
    UBoxComponent* ParentBox = GetBox(ParentActor);
    UBoxComponent* ChildBox = GetBox(ChildActor);
    ChildBox->AttachToComponent(ParentBox, FAttachmentTransformRules::KeepWorldTransform);
    const FVector ParentStart = ParentBox->GetComponentLocation();
    const FVector ChildStart = ChildBox->GetComponentLocation();

    UUBotPhysicsTestBinding* ChildBinding = AddBinding(*ChildActor, FVector(0.0, 1.0, 0.0));
    ChildBinding->Backend = SecondBackendName;
    UUBotPhysicsTestBinding* ParentBinding = AddBinding(*ParentActor, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("The child binds to the second backend"), ChildBinding->Bind()) || !TestTrue(TEXT("The parent binds to the first"), ParentBinding->Bind()))
    {
        return false;
    }
    TestEqual(TEXT("The child is simulated by the second backend"), Subsystem->FindBodyBackend(ChildBox), SecondBackendName);
    TestEqual(TEXT("The parent by the first"), Subsystem->FindBodyBackend(ParentBox), BackendName);
    FUBotPhysicsTestSolver* ParentSolver = FindTestSolver(*Subsystem);
    FUBotPhysicsTestSolver* ChildSolver = FindTestSolver(*Subsystem, SecondBackendName);
    if (!TestNotNull(TEXT("Both solvers exist"), ParentSolver) || !TestNotNull(TEXT("Both exist"), ChildSolver))
    {
        return false;
    }

    for (int32 Round = 0; Round < 3; ++Round)
    {
        Scope.RunFrames(7);
        if (!TestTrue(TEXT("Both solvers stepped"), ParentSolver->StepEndTimes.Num() > 0 && ChildSolver->StepEndTimes.Num() > 0))
        {
            return false;
        }
        const FVector ExpectedParent = ParentStart + FVector(1.0, 0.0, 0.0) * 100.0 * static_cast<double>(ParentSolver->StepEndTimes.Num()) * StepSeconds;
        const FVector ExpectedChild = ChildStart + FVector(0.0, 1.0, 0.0) * 100.0 * static_cast<double>(ChildSolver->StepEndTimes.Num()) * StepSeconds;
        TestTrue(*FString::Printf(TEXT("Round %d: the parent is at the pose of the first backend"), Round), ParentBox->GetComponentLocation().Equals(ExpectedParent, 1.e-3));
        TestTrue(*FString::Printf(TEXT("Round %d: the child is at the pose of the second backend, not carried along by its parent"), Round),
            ChildBox->GetComponentLocation().Equals(ExpectedChild, 1.e-3));
    }

    ChildBinding->Unbind();
    ParentBinding->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSteppingWriteBackEveryFrameTest, "UBotPhysics.Stepping.WriteBackEveryFrame", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSteppingWriteBackEveryFrameTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSteppingTests;

    // A step of 100 ms against frames of 16.7 ms: most frames run no step.
    constexpr double SlowStepSeconds = 0.1;
    FScopedTestBackend Backend(SlowStepSeconds);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    AActor* Actor = Scope.SpawnBoxActor(FVector(0.0, 0.0, 300.0), false);
    UBoxComponent* Box = GetBox(Actor);
    UUBotPhysicsTestBinding* Binding = AddBinding(*Actor, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("Bind succeeds"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }

    // Run to a frame that makes a step, and one more frame after it that makes none.
    int32 Guard = 0;
    while (Solver->StepEndTimes.Num() < 2 && Guard++ < 100)
    {
        Scope.Tick(FrameSeconds);
    }
    if (!TestTrue(TEXT("The solver stepped twice"), Solver->StepEndTimes.Num() >= 2))
    {
        return false;
    }
    FUBotBodyState State;
    if (!TestTrue(TEXT("The body has a state"), Subsystem->GetBodyState(Box, State)))
    {
        return false;
    }
    const FVector SolverPose = State.PositionMeters * 100.0;
    TestTrue(TEXT("The component is at the solver pose after the step frame"), Box->GetComponentLocation().Equals(SolverPose, 1.e-3));

    // Something else moves the component. The frame that follows runs no step, and still puts it back.
    Box->SetWorldLocation(SolverPose + FVector(0.0, 0.0, 500.0));
    const int32 StepsBefore = Solver->StepEndTimes.Num();
    Scope.Tick(FrameSeconds);
    if (!TestEqual(TEXT("Precondition: this frame ran no step"), Solver->StepEndTimes.Num(), StepsBefore))
    {
        return false;
    }
    TestTrue(TEXT("The component is back at the solver pose although its solver did not step"), Box->GetComponentLocation().Equals(SolverPose, 1.e-3));
    TestTrue(TEXT("Its velocity is still the solver's"), Box->ComponentVelocity.Equals(FVector(100.0, 0.0, 0.0), 1.e-3));

    // The same for the next frame without a step.
    Box->SetWorldLocation(SolverPose - FVector(0.0, 300.0, 0.0));
    Scope.Tick(FrameSeconds);
    if (Solver->StepEndTimes.Num() == StepsBefore)
    {
        TestTrue(TEXT("Frame after frame"), Box->GetComponentLocation().Equals(SolverPose, 1.e-3));
    }
    Binding->Unbind();
    return true;
}

#endif
