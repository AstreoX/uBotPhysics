#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"
#include "Serialization/ObjectAndNameAsStringProxyArchive.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotPhysicsModel.h"
#include "UBotPhysicsModelActor.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsModelDisplayQueriesTest, "UBotPhysics.Model.DisplayQueries", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsModelDisplayQueriesTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsTests;
    FScopedPhysicsTestWorld World(false);
    AUBotPhysicsModelActor* Actor = World.Get()->SpawnActor<AUBotPhysicsModelActor>();
    if (!TestNotNull(TEXT("Generic model actor spawns"), Actor)) { return false; }
    Actor->PhysicsBinding->bBindOnBeginPlay = false;
    UUBotPhysicsModelAsset* Asset = NewObject<UUBotPhysicsModelAsset>(Actor);
    Asset->Model = OneBodyModel();
    Asset->Model.Revision = FGuid::NewGuid();
    FUBotPhysicsModelGeometry& Geometry = Asset->Model.Geometries.AddDefaulted_GetRef();
    Geometry.Id = TEXT("exterior");
    Geometry.BodyId = TEXT("body");
    Geometry.Type = EUBotPhysicsGeometryType::Box;
    Geometry.Size = FVector(0.25);
    Geometry.bCollision = false;
    Actor->ModelAsset = Asset;
    TestTrue(TEXT("Common geometry builds without a backend"), Actor->RebuildModelDisplay());
    FHitResult Hit;
    FCollisionQueryParams ParametersForTrace(SCENE_QUERY_STAT(UBotPhysicsDisplay), false);
    TestTrue(TEXT("Visual-only geometry is visible to UE scene queries"), World.Get()->LineTraceSingleByChannel(
        Hit, FVector(-100.0, 0.0, 0.0), FVector(100.0, 0.0, 0.0), ECC_Visibility, ParametersForTrace));
    TestTrue(TEXT("Query hit belongs to the model actor"), Hit.GetActor() == Actor);
    UPrimitiveComponent* Display = Hit.GetComponent();
    if (TestNotNull(TEXT("The generated display provides the query hit"), Display))
    {
        TestEqual(TEXT("Visual-only geometry has query-only collision"), Display->GetCollisionEnabled(), ECollisionEnabled::QueryOnly);
        TestFalse(TEXT("Visual-only geometry has no physical collision"), CollisionEnabledHasPhysics(Display->GetCollisionEnabled()));
        TestFalse(TEXT("The display does not run a second rigid-body simulation"), Display->IsSimulatingPhysics());
    }
    TInlineComponentArray<UPrimitiveComponent*> Before;
    Actor->GetComponents(Before);
    TestTrue(TEXT("The generic display can be rebuilt"), Actor->RebuildModelDisplay());
    TInlineComponentArray<UPrimitiveComponent*> After;
    Actor->GetComponents(After);
    TestEqual(TEXT("Rebuild replaces components without accumulating proxies"), After.Num(), Before.Num());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsModelScaledInstanceTest, "UBotPhysics.Model.ScaledInstanceRejected", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsModelScaledInstanceTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsTests;
    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld World(false);
    AUBotPhysicsModelActor* Actor = World.Get()->SpawnActor<AUBotPhysicsModelActor>();
    Actor->PhysicsBinding->bBindOnBeginPlay = false;
    UUBotPhysicsModelAsset* Asset = NewObject<UUBotPhysicsModelAsset>(Actor);
    Asset->Model = OneBodyModel();
    FUBotPhysicsModelJoint& Joint = Asset->Model.Joints.AddDefaulted_GetRef();
    Joint.Id = TEXT("free"); Joint.BodyId = TEXT("body"); Joint.Type = EUBotPhysicsJointType::Free;
    Actor->ModelAsset = Asset;
    Actor->PhysicsBackend = BackendName;
    Actor->SetActorScale3D(FVector(2.0));
    TestTrue(TEXT("Display can be prepared before binding validation"), Actor->RebuildModelDisplay());
    TestFalse(TEXT("Scaled instance cannot silently diverge from native physical dimensions"), Actor->PhysicsBinding->Bind());
    TestTrue(TEXT("Error explains how to author physical scale"), Actor->PhysicsBinding->GetBindingError().ToString().Contains(TEXT("unit world scale")));
    TestEqual(TEXT("Invalid scale is rejected before solver creation"), Backend.Factory->CreateCalls, 0);
    TestEqual(TEXT("No model identity is claimed"), Actor->PhysicsBinding->GetModelInstanceId(), static_cast<int64>(INDEX_NONE));
    Actor->SetActorScale3D(FVector::OneVector);
    TestTrue(TEXT("Unit-scale model binds to the body-only backend"), Actor->PhysicsBinding->Bind());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsModelWorldOwnershipTest, "UBotPhysics.Model.CrossWorldOwnership", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsModelWorldOwnershipTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsTests;
    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld WorldA(false);
    FScopedPhysicsTestWorld WorldB(false);
    AActor* ActorB = nullptr;
    UUBotPhysicsTestBinding* BindingB = BindBox(WorldB, ActorB);
    if (!TestTrue(TEXT("Binding belongs to World B"), BindingB->Bind())) { return false; }
    const int64 InstanceId = BindingB->GetModelInstanceId();
    WorldB.GetSubsystem()->AdvanceTo(0.0);
    WorldB.GetSubsystem()->AdvanceTo(0.01);
    WorldA.GetSubsystem()->UnbindBodies(*BindingB);
    TestEqual(TEXT("Foreign World cannot clear binding state"), BindingB->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    TestEqual(TEXT("Foreign World cannot clear instance identity"), BindingB->GetModelInstanceId(), InstanceId);
    FText Error;
    TestFalse(TEXT("Foreign World cannot claim an already bound model"), WorldA.GetSubsystem()->BindBodies(*BindingB, Error));
    TestFalse(TEXT("Cross-world rejection has a reason"), Error.IsEmpty());
    TestEqual(TEXT("Rejection leaves actual owner's binding intact"), BindingB->GetBindingStatus(), EUBotPhysicsBindingStatus::Bound);
    FUBotBodyState State;
    TestTrue(TEXT("Actual owner's state remains readable"), WorldB.GetSubsystem()->GetBodyState(Cast<UPrimitiveComponent>(ActorB->GetRootComponent()), State));
    BindingB->Unbind();
    TestEqual(TEXT("The actual owner can still unbind"), BindingB->GetBindingStatus(), EUBotPhysicsBindingStatus::Unbound);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsModelPersistentBindingTest, "UBotPhysics.Model.PersistentGeneratedBindings", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsModelPersistentBindingTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsTests;
    FScopedTestBackend Backend;
    FScopedPhysicsTestWorld World(false);
    AUBotPhysicsModelActor* Actor = World.Get()->SpawnActor<AUBotPhysicsModelActor>();
    if (!TestNotNull(TEXT("Generic model actor spawns"), Actor)) { return false; }
    Actor->PhysicsBinding->bBindOnBeginPlay = false;
    UUBotPhysicsModelAsset* Asset = NewObject<UUBotPhysicsModelAsset>(Actor);
    Asset->Model = OneBodyModel();
    FUBotPhysicsModelBody OtherBody;
    OtherBody.Id = TEXT("other_body");
    OtherBody.LocalTransform.SetTranslation(FVector(1.0, 0.0, 0.0));
    Asset->Model.Bodies.Add(OtherBody);
    for (const FUBotPhysicsModelBody& Body : Asset->Model.Bodies)
    {
        FUBotPhysicsModelJoint& Joint = Asset->Model.Joints.AddDefaulted_GetRef();
        Joint.Id = Body.Id + TEXT(".free");
        Joint.BodyId = Body.Id;
        Joint.Type = EUBotPhysicsJointType::Free;
    }
    Actor->ModelAsset = Asset;
    Actor->PhysicsBackend = BackendName;
    if (!TestTrue(TEXT("Two model pivots are generated"), Actor->RebuildModelDisplay())
        || !TestEqual(TEXT("Both bodies have explicit mappings"), Actor->PhysicsBinding->Bodies.Num(), 2))
    {
        return false;
    }
    TestFalse(TEXT("The model actor root is intentionally not a physical body"), Actor->GetRootComponent()->IsA<UPrimitiveComponent>());
    TArray<FUBotPhysicsBodyBinding> LoadedBindings;
    TArray<UPrimitiveComponent*> ExpectedComponents;
    for (FUBotPhysicsBodyBinding& Mapping : Actor->PhysicsBinding->Bodies)
    {
        UPrimitiveComponent* Expected = Cast<UPrimitiveComponent>(Mapping.Component.GetComponent(Actor));
        if (!TestNotNull(TEXT("Generated mapping resolves its body pivot"), Expected)) { return false; }
        ExpectedComponents.Add(Expected);
        TestEqual(TEXT("The persistent path is relative to this actor"), Mapping.Component.PathToComponent, Expected->GetPathName(Actor));
        TestFalse(TEXT("The persistent path is not empty"), Mapping.Component.PathToComponent.IsEmpty());

        // Exercise the same reflected property boundary used by packages. OverrideComponent is not
        // reflected, so a fresh value read from the archive must resolve exclusively through the path.
        TArray<uint8> Bytes;
        {
            FMemoryWriter Memory(Bytes, true);
            FObjectAndNameAsStringProxyArchive Archive(Memory, false);
            FUBotPhysicsBodyBinding::StaticStruct()->SerializeItem(Archive, &Mapping, nullptr);
            TestFalse(TEXT("Binding serialization succeeds"), Archive.IsError());
        }
        FUBotPhysicsBodyBinding Loaded;
        {
            FMemoryReader Memory(Bytes, true);
            FObjectAndNameAsStringProxyArchive Archive(Memory, false);
            FUBotPhysicsBodyBinding::StaticStruct()->SerializeItem(Archive, &Loaded, nullptr);
            TestFalse(TEXT("Binding deserialization succeeds"), Archive.IsError());
        }
        TestFalse(TEXT("The transient override did not cross the property archive"), Loaded.Component.OverrideComponent.IsValid());
        TestEqual(TEXT("The stable model identity survived serialization"), Loaded.ModelBodyId, Mapping.ModelBodyId);
        TestTrue(TEXT("A loaded reference resolves the correct generated primitive"), Loaded.Component.GetComponent(Actor) == Expected);
        Mapping.Component.OverrideComponent.Reset();
        TestTrue(TEXT("The live mapping also resolves without its override cache"), Mapping.Component.GetComponent(Actor) == Expected);
        LoadedBindings.Add(MoveTemp(Loaded));
    }
    Actor->PhysicsBinding->Bodies = MoveTemp(LoadedBindings);
    TestTrue(TEXT("Persisted references complete model binding without any override caches"), Actor->PhysicsBinding->Bind());
    TestEqual(TEXT("Both persisted body references are bound"), Actor->PhysicsBinding->GetBoundComponents().Num(), 2);
    for (const UPrimitiveComponent* Component : ExpectedComponents)
    {
        TestTrue(TEXT("The bound source provides the generated body, not the scene root"), Actor->PhysicsBinding->ProvidesBodyState(Component));
    }
    Actor->PhysicsBinding->Unbind();
    return true;
}

#endif
