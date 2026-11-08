#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotPhysicsModel.h"
#include "UBotPhysicsSubsystem.h"
#include "UBotPhysicsTestTypes.h"
#include "UBotPhysicsTestWorld.h"

namespace
{
    UUBotPhysicsTestBinding* BindBox(UBotPhysicsTests::FScopedPhysicsTestWorld& World, AActor*& OutActor)
    {
        OutActor = World.SpawnBoxActor(FVector::ZeroVector, false);
        UUBotPhysicsTestBinding* Binding = NewObject<UUBotPhysicsTestBinding>(OutActor);
        Binding->Backend = UBotPhysicsTests::BackendName;
        Binding->bBindOnBeginPlay = false;
        Binding->VelocityMetersPerSecond = FVector(1.0, 0.0, 0.0);
        Binding->RegisterComponent();
        return Binding;
    }

    FUBotPhysicsModel OneBodyModel()
    {
        FUBotPhysicsModel Model;
        FUBotPhysicsModelBody& Body = Model.Bodies.AddDefaulted_GetRef();
        Body.Id = TEXT("body");
        return Model;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsModelValidationTest, "UBotPhysics.Model.Validation", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsModelValidationTest::RunTest(const FString& Parameters)
{
    FText Error;
    FUBotPhysicsModel Model = OneBodyModel();
    TestTrue(TEXT("A simple finite model is valid"), Model.Validate(Error));
    const FUBotPhysicsModelBody Duplicate = Model.Bodies[0];
    Model.Bodies.Add(Duplicate);
    TestFalse(TEXT("Duplicate identities are rejected"), Model.Validate(Error));
    Model.Bodies[1].Id = TEXT("BODY");
    TestTrue(TEXT("Case-distinct native names remain distinct"), Model.Validate(Error));
    Model.Bodies[1].Id = TEXT("child");
    Model.Bodies[1].ParentId = TEXT("body");
    Model.Bodies[0].ParentId = TEXT("child");
    TestFalse(TEXT("Parent cycles are rejected"), Model.Validate(Error));
    Model = OneBodyModel();
    Model.Bodies[0].MassKilograms = 0.0;
    Model.Bodies[0].InertiaKilogramMetersSquared = FVector::ZeroVector;
    TestTrue(TEXT("Massless structural bodies are deferred to the backend compiler"), Model.Validate(Error));
    Model = OneBodyModel();
    FUBotPhysicsModelJoint& Joint = Model.Joints.AddDefaulted_GetRef();
    Joint.Id = TEXT("ball");
    Joint.BodyId = TEXT("body");
    Joint.Type = EUBotPhysicsJointType::Ball;
    Joint.InitialPosition = { 0.0, 0.0, 0.0, 1.0 };
    Joint.InitialVelocity = { 0.0, 0.0, 0.0 };
    TestTrue(TEXT("Ball position and velocity have independent dimensions"), Model.Validate(Error));
    Joint.InitialVelocity.Add(0.0);
    TestFalse(TEXT("Treating nq as nv is rejected"), Model.Validate(Error));
    Model = OneBodyModel();
    Model.Bodies[0].LocalTransform.SetScale3D(FVector(2.0));
    TestFalse(TEXT("Unbaked model scale is rejected"), Model.Validate(Error));
    Model = OneBodyModel();
    Model.Bodies[0].Id = TEXT("world");
    Model.Bodies[0].MassKilograms = 0.0;
    Model.Bodies[0].InertiaKilogramMetersSquared = FVector::ZeroVector;
    FUBotPhysicsModelBody FreeBody;
    FreeBody.Id = TEXT("free_body"); FreeBody.ParentId = TEXT("world");
    Model.Bodies.Add(FreeBody);
    FUBotPhysicsModelJoint FreeJoint;
    FreeJoint.Id = TEXT("free_joint"); FreeJoint.BodyId = TEXT("free_body"); FreeJoint.Type = EUBotPhysicsJointType::Free;
    Model.Joints = { FreeJoint };
    TestTrue(TEXT("A free body may refer to an explicit static model frame"), Model.Validate(Error));
    FUBotPhysicsModelBuilder Builder;
    Builder.Model = OneBodyModel();
    FUBotPhysicsModel Built;
    TestTrue(TEXT("Builder validates and publishes a revision"), Builder.Finalize(Built, Error));
    TestTrue(TEXT("Revision is valid"), Built.Revision.IsValid());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsModelContinuousBudgetTest, "UBotPhysics.Model.ContinuousBudget", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsModelContinuousBudgetTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsTests;
    FScopedTestBackend Backend(0.01);
    FScopedPhysicsTestWorld World(false);
    UUBotPhysicsSubsystem* Subsystem = World.GetSubsystem();
    AActor* Actor = nullptr;
    UUBotPhysicsTestBinding* Binding = BindBox(World, Actor);
    if (!TestTrue(TEXT("Binds"), Binding->Bind())) { return false; }
    auto* Solver = static_cast<FUBotPhysicsTestSolver*>(Subsystem->FindSolver(BackendName));
    Solver->OverrunPolicy = EUBotPhysicsOverrunPolicy::Suspend;
    Subsystem->SetMaxStepsPerFrame(2);
    Subsystem->AdvanceTo(0.0);
    Subsystem->AdvanceTo(0.05);
    FText Error;
    TestEqual(TEXT("Budget exhaustion suspends continuous time"), Subsystem->GetBackendStatus(BackendName, Error), EUBotPhysicsBackendStatus::Overrun);
    TestEqual(TEXT("Exactly the budget was integrated"), Solver->StepEndTimes.Num(), 2);
    TestEqual(TEXT("Excess grid points were not dropped"), Subsystem->GetDroppedSteps(BackendName), static_cast<int64>(0));
    FUBotBodyState State;
    TestTrue(TEXT("The last successful state remains readable"), Subsystem->GetBodyState(Cast<UPrimitiveComponent>(Actor->GetRootComponent()), State));
    TestTrue(TEXT("Timestamp is actual last integrated time"), FMath::IsNearlyEqual(State.TimestampSeconds, 0.02, 1.e-12));
    TestTrue(TEXT("Position follows two integrated steps, not the frame time"), FMath::IsNearlyEqual(State.PositionMeters.X, 0.02, 1.e-12));
    Subsystem->AdvanceTo(0.1);
    TestEqual(TEXT("Suspended solver does not silently continue"), Solver->StepEndTimes.Num(), 2);
    TestTrue(TEXT("Explicit whole-backend reset succeeds"), Subsystem->ResetBackend(BackendName, Error));
    TestEqual(TEXT("Reset clears suspension"), Subsystem->GetBackendStatus(BackendName, Error), EUBotPhysicsBackendStatus::Ready);
    TestFalse(TEXT("No pre-reset state is relabeled into the new epoch"), Subsystem->GetBodyState(Cast<UPrimitiveComponent>(Actor->GetRootComponent()), State));
    Subsystem->AdvanceTo(0.01);
    TestEqual(TEXT("First step of the new epoch executes"), Solver->StepEndTimes.Num(), 1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsModelFailurePublicationTest, "UBotPhysics.Model.FailurePublication", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsModelFailurePublicationTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsTests;
    FScopedTestBackend Backend(0.01);
    FScopedPhysicsTestWorld World(false);
    UUBotPhysicsSubsystem* Subsystem = World.GetSubsystem();
    AActor* Actor = nullptr;
    UUBotPhysicsTestBinding* Binding = BindBox(World, Actor);
    if (!TestTrue(TEXT("Binds"), Binding->Bind())) { return false; }
    auto* Solver = static_cast<FUBotPhysicsTestSolver*>(Subsystem->FindSolver(BackendName));
    Subsystem->AdvanceTo(0.0);
    Subsystem->AdvanceTo(0.01);
    Solver->bFailStep = true;
    Subsystem->AdvanceTo(0.02);
    FText Error;
    TestEqual(TEXT("Failed integration suspends backend"), Subsystem->GetBackendStatus(BackendName, Error), EUBotPhysicsBackendStatus::Failed);
    FUBotBodyState State;
    TestTrue(TEXT("Published state survives failure"), Subsystem->GetBodyState(Cast<UPrimitiveComponent>(Actor->GetRootComponent()), State));
    TestTrue(TEXT("Failed step did not advance timestamp"), FMath::IsNearlyEqual(State.TimestampSeconds, 0.01, 1.e-12));
    TestEqual(TEXT("Failed step did not advance integrator"), Solver->StepEndTimes.Num(), 1);
    TestTrue(TEXT("Reset permits further validation"), Subsystem->ResetBackend(BackendName, Error));
    // A scene update failure must be propagated before another integration step.
    AActor* Collider = World.SpawnBoxActor(FVector(100.0, 0.0, 0.0), true);
    Solver->bFailScene = true;
    Subsystem->AdvanceTo(0.01);
    TestEqual(TEXT("Scene failure also suspends"), Subsystem->GetBackendStatus(BackendName, Error), EUBotPhysicsBackendStatus::Failed);
    TestEqual(TEXT("No step runs against an obsolete scene"), Solver->StepEndTimes.Num(), 0);
    return Collider != nullptr;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsModelUnsupportedAtomicTest, "UBotPhysics.Model.UnsupportedAtomic", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsModelUnsupportedAtomicTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsTests;
    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld World(false);
    AActor* Actor = nullptr;
    UUBotPhysicsTestBinding* Binding = BindBox(World, Actor);
    UUBotPhysicsModelAsset* Asset = NewObject<UUBotPhysicsModelAsset>(Binding);
    Asset->Model = OneBodyModel();
    FUBotPhysicsModelJoint& Joint = Asset->Model.Joints.AddDefaulted_GetRef();
    Joint.Id = TEXT("hinge"); Joint.BodyId = TEXT("body");
    Binding->Model = Asset;
    TestFalse(TEXT("Body-only default cannot silently flatten articulation"), Binding->Bind());
    auto* Solver = static_cast<FUBotPhysicsTestSolver*>(World.GetSubsystem()->FindSolver(BackendName));
    TestEqual(TEXT("No body was handed to the unsupported backend"), Solver->GetBodyCount(), 0);
    TestEqual(TEXT("Failed model owns no instance identity"), Binding->GetModelInstanceId(), static_cast<int64>(INDEX_NONE));
    return true;
}

#endif
