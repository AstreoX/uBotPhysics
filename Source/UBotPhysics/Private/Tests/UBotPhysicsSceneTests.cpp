#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodySetup.h"
#include "UBotPhysics.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotPhysicsLibrary.h"
#include "UBotPhysicsSceneShapes.h"
#include "UBotPhysicsSubsystem.h"
#include "UBotPhysicsTestTypes.h"
#include "UBotPhysicsTestWorld.h"

// The scene a backend sees: which components reach a solver as static and kinematic bodies, what an update
// holds, and when it comes. The test backend records every UpdateScene call and keeps the scene the updates
// add up to, so the tests read what a solver would know.
namespace UBotPhysicsSceneTests
{
    using namespace UBotPhysicsTests;

    constexpr double StepSeconds = 0.01;

    /** Tolerance for positions and sizes in meters. */
    constexpr double Tolerance = 1.e-6;

    UBoxComponent* GetBox(const AActor* Actor)
    {
        return Actor != nullptr ? Cast<UBoxComponent>(Actor->GetRootComponent()) : nullptr;
    }

    FUBotPhysicsTestSolver* FindTestSolver(const UUBotPhysicsSubsystem& Subsystem, FName Backend = BackendName)
    {
        return static_cast<FUBotPhysicsTestSolver*>(Subsystem.FindSolver(Backend));
    }

    UUBotPhysicsTestBinding* AddBinding(AActor& Actor, FName Backend = BackendName, const FVector& VelocityMetersPerSecond = FVector::ZeroVector)
    {
        UUBotPhysicsTestBinding* Binding = NewObject<UUBotPhysicsTestBinding>(&Actor);
        Binding->Backend = Backend;
        Binding->bBindOnBeginPlay = false;
        Binding->VelocityMetersPerSecond = VelocityMetersPerSecond;
        Binding->RegisterComponent();
        return Binding;
    }

    // An actor whose root is a box that blocks everything and that nobody simulates: a candidate for the scene of a
    // backend. HalfExtentCm are the half extents of the box.
    AActor* SpawnCollider(const FScopedPhysicsTestWorld& Scope, const FVector& LocationCm, EComponentMobility::Type Mobility,
        const FVector& HalfExtentCm = FVector(25.0), UPhysicalMaterial* Material = nullptr)
    {
        AActor* Actor = Scope.Get()->SpawnActor<AActor>();
        UBoxComponent* Box = NewObject<UBoxComponent>(Actor, TEXT("Box"));
        Box->SetBoxExtent(HalfExtentCm, false);
        Box->SetMobility(Mobility);
        Box->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
        if (Material != nullptr)
        {
            Box->SetPhysMaterialOverride(Material);
        }
        Actor->SetRootComponent(Box);
        Box->SetWorldLocation(LocationCm);
        Box->RegisterComponent();
        return Actor;
    }

    // A component of its own actor, placed at WorldTransform, without collision: GetCollisionShapes does not look at
    // collision settings, and a body with no collision has no physics state. Setup runs before it is registered.
    template <typename ComponentType>
    ComponentType* SpawnShapeComponent(const FScopedPhysicsTestWorld& Scope, const FTransform& WorldTransform, TFunctionRef<void(ComponentType&)> Setup)
    {
        AActor* Actor = Scope.Get()->SpawnActor<AActor>();
        ComponentType* Component = NewObject<ComponentType>(Actor);
        Setup(*Component);
        Component->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
        Actor->SetRootComponent(Component);
        Component->SetWorldTransform(WorldTransform);
        Component->RegisterComponent();
        return Component;
    }

    // An actor whose root is a component of the given class that blocks everything.
    template <typename ComponentType>
    AActor* SpawnBlockingActor(const FScopedPhysicsTestWorld& Scope)
    {
        AActor* Actor = Scope.Get()->SpawnActor<AActor>();
        ComponentType* Component = NewObject<ComponentType>(Actor);
        Component->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
        Actor->SetRootComponent(Component);
        Component->RegisterComponent();
        return Actor;
    }

    const FUBotPhysicsSceneBody* FindAdded(const FUBotPhysicsSceneUpdate& Update, const UPrimitiveComponent* Component)
    {
        return Update.Added.FindByPredicate([Component](const FUBotPhysicsSceneBody& Body)
        {
            return Body.Component.Get() == Component;
        });
    }

    const FUBotPhysicsScenePose* FindPose(const FUBotPhysicsSceneUpdate& Update, int64 Id)
    {
        return Update.KinematicPoses.FindByPredicate([Id](const FUBotPhysicsScenePose& Pose)
        {
            return Pose.Id == Id;
        });
    }

    bool IsIdentity(const FTransform& Transform)
    {
        return Transform.GetTranslation().IsNearlyZero(Tolerance) && Transform.GetRotation().Equals(FQuat::Identity, Tolerance)
            && Transform.GetScale3D().Equals(FVector::OneVector, Tolerance);
    }

    // The solver must have been told the truth: no removal of a body it did not know, no second addition, no pose of
    // a body it did not know.
    void ExpectCleanProtocol(FAutomationTestBase& Test, const TCHAR* Case, const FUBotPhysicsTestSolver& Solver)
    {
        Test.TestEqual(*FString::Printf(TEXT("%s: no removal of a body the solver did not know"), Case), Solver.UnknownRemovals, 0);
        Test.TestEqual(*FString::Printf(TEXT("%s: no addition of a body the solver already knew"), Case), Solver.DuplicateAdds, 0);
        Test.TestEqual(*FString::Printf(TEXT("%s: no pose of a body the solver did not know"), Case), Solver.UnknownPoses, 0);
    }

    // Shapes of Setup the way the engine scales them: the test's own conversion that GetCollisionShapes is compared with.
    void ExpectedShapesOf(const UBodySetup& Setup, const FVector& Scale3D, TArray<FUBotPhysicsShape>& OutShapes)
    {
        OutShapes.Reset();
        for (const FKSphereElem& Element : Setup.AggGeom.SphereElems)
        {
            const FKSphereElem Scaled = Element.GetFinalScaled(Scale3D, FTransform::Identity);
            FUBotPhysicsShape& Shape = OutShapes.AddDefaulted_GetRef();
            Shape.Type = EUBotPhysicsShapeType::Sphere;
            Shape.LocalTransform = FTransform(Scaled.Center * 0.01);
            Shape.Size = FVector(Scaled.Radius * 0.01, 0.0, 0.0);
        }
        for (const FKBoxElem& Element : Setup.AggGeom.BoxElems)
        {
            // X, Y and Z are full extents.
            const FKBoxElem Scaled = Element.GetFinalScaled(Scale3D, FTransform::Identity);
            FUBotPhysicsShape& Shape = OutShapes.AddDefaulted_GetRef();
            Shape.Type = EUBotPhysicsShapeType::Box;
            Shape.LocalTransform = FTransform(Scaled.Rotation.Quaternion(), Scaled.Center * 0.01);
            Shape.Size = FVector(Scaled.X, Scaled.Y, Scaled.Z) * 0.005;
        }
        for (const FKSphylElem& Element : Setup.AggGeom.SphylElems)
        {
            const FKSphylElem Scaled = Element.GetFinalScaled(Scale3D, FTransform::Identity);
            FUBotPhysicsShape& Shape = OutShapes.AddDefaulted_GetRef();
            Shape.Type = EUBotPhysicsShapeType::Capsule;
            Shape.LocalTransform = FTransform(Scaled.Rotation.Quaternion(), Scaled.Center * 0.01);
            Shape.Size = FVector(Scaled.Radius * 0.01, 0.0, Scaled.Length * 0.005);
        }
        for (const FKConvexElem& Element : Setup.AggGeom.ConvexElems)
        {
            const FTransform ElementTransform = Element.GetTransform();
            FUBotPhysicsShape& Shape = OutShapes.AddDefaulted_GetRef();
            Shape.Type = EUBotPhysicsShapeType::Convex;
            for (const FVector& Vertex : Element.VertexData)
            {
                Shape.Vertices.Add(Scale3D * ElementTransform.TransformPosition(Vertex) * 0.01);
            }
        }
    }

    void ExpectSameShapes(FAutomationTestBase& Test, const FString& Case, const TArray<FUBotPhysicsShape>& Actual, const TArray<FUBotPhysicsShape>& Expected)
    {
        if (!Test.TestEqual(*FString::Printf(TEXT("%s: number of shapes"), *Case), Actual.Num(), Expected.Num()))
        {
            return;
        }
        for (int32 Index = 0; Index < Actual.Num(); ++Index)
        {
            const FUBotPhysicsShape& Shape = Actual[Index];
            const FUBotPhysicsShape& Wanted = Expected[Index];
            const FString Name = FString::Printf(TEXT("%s: shape %d"), *Case, Index);
            Test.TestTrue(*(Name + TEXT(" has the type")), Shape.Type == Wanted.Type);
            Test.TestTrue(*(Name + TEXT(" has the size")), Shape.Size.Equals(Wanted.Size, Tolerance));
            Test.TestTrue(*(Name + TEXT(" is where the engine puts it")), Shape.LocalTransform.GetTranslation().Equals(Wanted.LocalTransform.GetTranslation(), Tolerance));
            Test.TestTrue(*(Name + TEXT(" is turned the way the engine turns it")), Shape.LocalTransform.GetRotation().Equals(Wanted.LocalTransform.GetRotation(), 1.e-5));
            if (Test.TestEqual(*(Name + TEXT(" has the number of vertices")), Shape.Vertices.Num(), Wanted.Vertices.Num()))
            {
                int32 Off = 0;
                for (int32 VertexIndex = 0; VertexIndex < Shape.Vertices.Num(); ++VertexIndex)
                {
                    Off += Shape.Vertices[VertexIndex].Equals(Wanted.Vertices[VertexIndex], Tolerance) ? 0 : 1;
                }
                Test.TestEqual(*(Name + TEXT(" has the vertices")), Off, 0);
            }
        }
    }

    // Triangles of a mesh around the origin that face outward: the normal of (A, B, C) points away from the origin.
    int32 CountOutwardTriangles(const TArray<FVector>& Vertices, const TArray<int32>& Indices)
    {
        int32 Outward = 0;
        for (int32 First = 0; First + 2 < Indices.Num(); First += 3)
        {
            const FVector& A = Vertices[Indices[First]];
            const FVector& B = Vertices[Indices[First + 1]];
            const FVector& C = Vertices[Indices[First + 2]];
            const FVector Normal = FVector::CrossProduct(B - A, C - A);
            Outward += FVector::DotProduct(Normal, (A + B + C) / 3.0) > 0.0 ? 1 : 0;
        }
        return Outward;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSceneCollisionShapesTest, "UBotPhysics.Scene.CollisionShapes", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSceneCollisionShapesTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSceneTests;

    FScopedPhysicsTestWorld Scope;
    if (!TestNotNull(TEXT("The test world exists"), Scope.Get()))
    {
        return false;
    }
    TestEqual(TEXT("The tag that keeps a component out of the scene"), UBot::Physics::NoSceneTag.ToString(), FString(TEXT("UBotPhysics.NoScene")));

    const FVector Location(100.0, -200.0, 300.0);
    const FRotator Rotation(10.0, 20.0, 30.0);
    const FVector Scale(1.25, 1.5, 2.0);
    const FVector MirroredScale(-1.25, 1.5, 2.0);
    const FTransform WorldTransform(Rotation, Location, Scale);

    TArray<FUBotPhysicsShape> Shapes;
    int32 Skipped = -1;

    // 1. Box: half extents are the extent of the component times the scale (in magnitude), in meters; the position
    //    and rotation of the component are not part of the shapes.
    UBoxComponent* Box = SpawnShapeComponent<UBoxComponent>(Scope, WorldTransform, [](UBoxComponent& Component)
    {
        Component.SetBoxExtent(FVector(50.0, 30.0, 20.0), false);
    });
    if (TestTrue(TEXT("Box: shapes are exported"), UBot::Physics::GetCollisionShapes(*Box, Shapes, &Skipped)) && TestEqual(TEXT("Box: one shape"), Shapes.Num(), 1))
    {
        TestEqual(TEXT("Box: nothing is skipped"), Skipped, 0);
        TestTrue(TEXT("Box: the type"), Shapes[0].Type == EUBotPhysicsShapeType::Box);
        TestTrue(TEXT("Box: half extents in meters, scaled"), Shapes[0].Size.Equals(FVector(0.625, 0.45, 0.4), Tolerance));
        TestTrue(TEXT("Box: the shape frame is the body frame"), IsIdentity(Shapes[0].LocalTransform));
        TestTrue(TEXT("Box: no mesh data"), Shapes[0].Vertices.IsEmpty() && Shapes[0].Indices.IsEmpty());
    }
    TArray<FUBotPhysicsShape> Expected;
    if (UBodySetup* BoxSetup = Box->GetBodySetup())
    {
        ExpectedShapesOf(*BoxSetup, Scale, Expected);
        ExpectSameShapes(*this, TEXT("Box against the engine's scaling"), Shapes, Expected);
    }

    // A mirroring scale does not make the box smaller or negative.
    Box->SetWorldScale3D(MirroredScale);
    if (TestTrue(TEXT("Mirrored box: shapes are exported"), UBot::Physics::GetCollisionShapes(*Box, Shapes)) && TestEqual(TEXT("Mirrored box: one shape"), Shapes.Num(), 1))
    {
        TestTrue(TEXT("Mirrored box: the same half extents"), Shapes[0].Size.Equals(FVector(0.625, 0.45, 0.4), Tolerance));
    }
    Box->SetWorldScale3D(Scale);

    // 2. Sphere: the radius follows the smallest scale.
    USphereComponent* Sphere = SpawnShapeComponent<USphereComponent>(Scope, WorldTransform, [](USphereComponent& Component)
    {
        Component.SetSphereRadius(40.0f, false);
    });
    if (TestTrue(TEXT("Sphere: shapes are exported"), UBot::Physics::GetCollisionShapes(*Sphere, Shapes, &Skipped)) && TestEqual(TEXT("Sphere: one shape"), Shapes.Num(), 1))
    {
        TestEqual(TEXT("Sphere: nothing is skipped"), Skipped, 0);
        TestTrue(TEXT("Sphere: the type"), Shapes[0].Type == EUBotPhysicsShapeType::Sphere);
        TestTrue(TEXT("Sphere: radius in meters, with the smallest scale"), FMath::IsNearlyEqual(Shapes[0].Size.X, 0.4 * 1.25, Tolerance));
        TestTrue(TEXT("Sphere: at the body origin"), IsIdentity(Shapes[0].LocalTransform));
    }
    if (UBodySetup* SphereSetup = Sphere->GetBodySetup())
    {
        ExpectedShapesOf(*SphereSetup, Scale, Expected);
        ExpectSameShapes(*this, TEXT("Sphere against the engine's scaling"), Shapes, Expected);
    }

    // 3. Capsule: radius 20 cm and half height 50 cm. The radius follows the larger of X and Y, the half length Z.
    UCapsuleComponent* Capsule = SpawnShapeComponent<UCapsuleComponent>(Scope, WorldTransform, [](UCapsuleComponent& Component)
    {
        Component.SetCapsuleSize(20.0f, 50.0f, false);
    });
    if (TestTrue(TEXT("Capsule: shapes are exported"), UBot::Physics::GetCollisionShapes(*Capsule, Shapes, &Skipped)) && TestEqual(TEXT("Capsule: one shape"), Shapes.Num(), 1))
    {
        TestEqual(TEXT("Capsule: nothing is skipped"), Skipped, 0);
        TestTrue(TEXT("Capsule: the type"), Shapes[0].Type == EUBotPhysicsShapeType::Capsule);
        // Radius 20 * 1.5 = 30 cm; the whole half length 50 * 2 = 100 cm, so the segment between the sphere centers is 140 cm.
        TestTrue(TEXT("Capsule: radius and half the distance between the sphere centers, in meters"), Shapes[0].Size.Equals(FVector(0.3, 0.0, 0.7), Tolerance));
        TestTrue(TEXT("Capsule: the axis is Z of the body"), IsIdentity(Shapes[0].LocalTransform));
    }
    if (UBodySetup* CapsuleSetup = Capsule->GetBodySetup())
    {
        ExpectedShapesOf(*CapsuleSetup, Scale, Expected);
        ExpectSameShapes(*this, TEXT("Capsule against the engine's scaling"), Shapes, Expected);
    }

    // 4. A body setup that holds other things next to the box: a hull with a transform of its own, a hull too small to
    //    be one and a tapered capsule. The hull is moved by its own transform first and scaled per axis after; the other two
    //    are left out and counted. The setup must be the box's own: a shape component shares the setup of its class
    //    as long as its extent is the default one.
    UBodySetup* OwnSetup = Box->GetBodySetup();
    if (TestTrue(TEXT("The box has a body setup of its own"), OwnSetup != nullptr && OwnSetup->GetOuter() == Box))
    {
        UBodySetup* BoxSetup = OwnSetup;
        BoxSetup->CollisionTraceFlag = CTF_UseComplexAsSimple;
        TestTrue(TEXT("Complex collision as simple without triangle meshes: the simple shapes are what Chaos simulates"),
            UBot::Physics::GetCollisionShapes(*Box, Shapes) && Shapes.Num() == 1 && Shapes[0].Type == EUBotPhysicsShapeType::Box);
        BoxSetup->CollisionTraceFlag = CTF_UseSimpleAsComplex;

        FKConvexElem Hull;
        Hull.VertexData = { FVector(50.0, 0.0, 0.0), FVector(-50.0, 0.0, 0.0), FVector(0.0, 50.0, 0.0), FVector(0.0, -50.0, 0.0), FVector(0.0, 0.0, 50.0), FVector(0.0, 0.0, -50.0) };
        Hull.SetTransform(FTransform(FRotator(0.0, 45.0, 0.0), FVector(10.0, 20.0, 30.0), FVector(2.0, 1.0, 1.0)));
        BoxSetup->AggGeom.ConvexElems.Add(Hull);
        FKConvexElem TooSmall;
        TooSmall.VertexData = { FVector::ZeroVector, FVector(10.0, 0.0, 0.0), FVector(0.0, 10.0, 0.0) };
        BoxSetup->AggGeom.ConvexElems.Add(TooSmall);
        BoxSetup->AggGeom.TaperedCapsuleElems.Add(FKTaperedCapsuleElem());

        if (TestTrue(TEXT("Hull: shapes are exported"), UBot::Physics::GetCollisionShapes(*Box, Shapes, &Skipped)) && TestEqual(TEXT("Hull: the box and the hull"), Shapes.Num(), 2))
        {
            TestEqual(TEXT("Hull: the too small hull and the tapered capsule are counted"), Skipped, 2);
            TestTrue(TEXT("Hull: the box comes first"), Shapes[0].Type == EUBotPhysicsShapeType::Box);
            TestTrue(TEXT("Hull: the hull is a convex shape"), Shapes[1].Type == EUBotPhysicsShapeType::Convex);
            TestTrue(TEXT("Hull: in the body frame"), IsIdentity(Shapes[1].LocalTransform));
            if (TestEqual(TEXT("Hull: six vertices"), Shapes[1].Vertices.Num(), 6))
            {
                // (50, 0, 0): scaled by the hull's (2, 1, 1) = (100, 0, 0), turned 45 degrees = (70.71, 70.71, 0), moved by
                // (10, 20, 30) = (80.71, 90.71, 30), scaled by the body's (1.25, 1.5, 2) = (100.89, 136.07, 60) cm.
                TestTrue(TEXT("Hull: the first vertex, in meters"), Shapes[1].Vertices[0].Equals(FVector(1.008883, 1.360660, 0.6), 1.e-4));
            }
            ExpectedShapesOf(*BoxSetup, Scale, Expected);
            if (TestEqual(TEXT("Hull: the engine's list holds the box and both hulls"), Expected.Num(), 3))
            {
                // The hull that is too small is not exported.
                Expected.RemoveAt(2);
                ExpectSameShapes(*this, TEXT("Hull against the engine's scaling"), Shapes, Expected);
            }
        }
        BoxSetup->AggGeom.ConvexElems.Reset();
        BoxSetup->AggGeom.TaperedCapsuleElems.Reset();
    }

    // 5. The engine's cube and cylinder: whatever simple collision they have, scaled the way the engine scales it, with a
    //    scale that mirrors and one that does not.
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    UStaticMesh* Cylinder = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    if (!TestNotNull(TEXT("The engine's cube is available"), Cube) || !TestNotNull(TEXT("The engine's cylinder is available"), Cylinder))
    {
        return false;
    }
    UStaticMeshComponent* PlainCube = nullptr;
    for (UStaticMesh* Mesh : { Cube, Cylinder })
    {
        TArray<FUBotPhysicsShape> ByScale[2];
        const FVector MeshScales[2] = { Scale, MirroredScale };
        for (int32 ScaleIndex = 0; ScaleIndex < 2; ++ScaleIndex)
        {
            const FString Case = FString::Printf(TEXT("%s with scale %s"), *Mesh->GetName(), *MeshScales[ScaleIndex].ToString());
            UStaticMeshComponent* Component = SpawnShapeComponent<UStaticMeshComponent>(Scope, FTransform(Rotation, Location, MeshScales[ScaleIndex]),
                [Mesh](UStaticMeshComponent& NewComponent)
                {
                    NewComponent.SetStaticMesh(Mesh);
                });
            if (Mesh == Cube && ScaleIndex == 0)
            {
                PlainCube = Component;
            }

            const UBodySetup* Setup = Mesh->GetBodySetup();
            if (!TestNotNull(*FString::Printf(TEXT("%s: the mesh has a body setup"), *Case), Setup))
            {
                continue;
            }
            if (Setup->GetCollisionTraceFlag() == CTF_UseComplexAsSimple && !Setup->TriMeshGeometries.IsEmpty())
            {
                AddInfo(FString::Printf(TEXT("%s: complex collision as simple, left to the triangle mesh test"), *Case));
                continue;
            }
            ExpectedShapesOf(*Setup, MeshScales[ScaleIndex], Expected);
            if (TestTrue(*FString::Printf(TEXT("%s: has simple collision"), *Case), Expected.Num() > 0)
                && TestTrue(*FString::Printf(TEXT("%s: shapes are exported"), *Case), UBot::Physics::GetCollisionShapes(*Component, ByScale[ScaleIndex], &Skipped)))
            {
                ExpectSameShapes(*this, Case, ByScale[ScaleIndex], Expected);
            }
        }

        // The mirrored hull is the other one with X negated.
        if (ByScale[0].Num() == ByScale[1].Num())
        {
            for (int32 ShapeIndex = 0; ShapeIndex < ByScale[0].Num(); ++ShapeIndex)
            {
                const FUBotPhysicsShape& Straight = ByScale[0][ShapeIndex];
                const FUBotPhysicsShape& Mirrored = ByScale[1][ShapeIndex];
                if (Straight.Type != EUBotPhysicsShapeType::Convex || Straight.Vertices.Num() != Mirrored.Vertices.Num())
                {
                    continue;
                }
                int32 Off = 0;
                for (int32 VertexIndex = 0; VertexIndex < Straight.Vertices.Num(); ++VertexIndex)
                {
                    const FVector& A = Straight.Vertices[VertexIndex];
                    const FVector& B = Mirrored.Vertices[VertexIndex];
                    Off += FMath::IsNearlyEqual(A.X, -B.X, Tolerance) && FMath::IsNearlyEqual(A.Y, B.Y, Tolerance) && FMath::IsNearlyEqual(A.Z, B.Z, Tolerance) ? 0 : 1;
                }
                TestEqual(*FString::Printf(TEXT("%s: the mirrored hull has X negated"), *Mesh->GetName()), Off, 0);
            }
        }
    }

    // 6. Types that are not exported: instanced static meshes and skinned meshes, even with a mesh that has collision.
    {
        AActor* Actor = Scope.Get()->SpawnActor<AActor>();
        UInstancedStaticMeshComponent* Instanced = NewObject<UInstancedStaticMeshComponent>(Actor);
        Instanced->SetStaticMesh(Cube);
        USkeletalMeshComponent* Skinned = NewObject<USkeletalMeshComponent>(Actor);
        TestTrue(TEXT("Plain: the cube on a static mesh component is exported"), PlainCube != nullptr && UBot::Physics::GetCollisionShapes(*PlainCube, Shapes));
        TestFalse(TEXT("Instanced: the same mesh on an instanced static mesh component is not"), UBot::Physics::GetCollisionShapes(*Instanced, Shapes, &Skipped));
        TestTrue(TEXT("Instanced: no shapes, nothing counted"), Shapes.IsEmpty() && Skipped == 0);
        TestFalse(TEXT("Skinned: a skeletal mesh component is not"), UBot::Physics::GetCollisionShapes(*Skinned, Shapes, &Skipped));
        TestTrue(TEXT("Skinned: no shapes, nothing counted"), Shapes.IsEmpty() && Skipped == 0);
    }

    // 7. Triangle meshes, through the conversion that turns Chaos triangles into a shape (cooked meshes are not made
    //    in the test): a cube of 100 cm around the origin as 12 triangles facing out. A scale that mirrors an odd
    //    number of axes turns them inside out, so their winding is swapped.
    {
        TArray<FVector> Vertices;
        TArray<int32> Indices;
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            for (int32 Side = 0; Side < 2; ++Side)
            {
                const double Sign = Side == 0 ? 1.0 : -1.0;
                FVector Normal = FVector::ZeroVector;
                FVector U = FVector::ZeroVector;
                FVector V = FVector::ZeroVector;
                Normal[Axis] = Sign;
                U[(Axis + 1) % 3] = 1.0;
                V[(Axis + 2) % 3] = 1.0;
                const int32 Base = Vertices.Num();
                Vertices.Add((Normal - U - V) * 50.0);
                Vertices.Add((Normal + U - V) * 50.0);
                Vertices.Add((Normal + U + V) * 50.0);
                Vertices.Add((Normal - U + V) * 50.0);
                if (Sign > 0.0)
                {
                    Indices.Append({ Base, Base + 1, Base + 2, Base, Base + 2, Base + 3 });
                }
                else
                {
                    Indices.Append({ Base, Base + 2, Base + 1, Base, Base + 3, Base + 2 });
                }
            }
        }

        const FVector MeshScales[4] = { FVector(1.0), FVector(2.0, 3.0, 0.5), FVector(-1.0, 2.0, 0.5), FVector(-1.0, -2.0, 0.5) };
        const bool bMirrors[4] = { false, false, true, false };
        for (int32 Index = 0; Index < 4; ++Index)
        {
            const FString Case = FString::Printf(TEXT("Triangle mesh with scale %s"), *MeshScales[Index].ToString());
            FUBotPhysicsShape Mesh;
            if (!TestTrue(*(Case + TEXT(": a shape is made")), UBot::Physics::Private::MakeTriangleMeshShape(Vertices, Indices, MeshScales[Index], Mesh)))
            {
                continue;
            }
            TestTrue(*(Case + TEXT(": the type")), Mesh.Type == EUBotPhysicsShapeType::TriangleMesh);
            TestTrue(*(Case + TEXT(": the shape frame is the body frame")), IsIdentity(Mesh.LocalTransform));
            TestEqual(*(Case + TEXT(": the vertices")), Mesh.Vertices.Num(), Vertices.Num());
            TestEqual(*(Case + TEXT(": three indices per triangle")), Mesh.Indices.Num(), Indices.Num());
            int32 Off = 0;
            for (int32 VertexIndex = 0; VertexIndex < FMath::Min(Mesh.Vertices.Num(), Vertices.Num()); ++VertexIndex)
            {
                Off += Mesh.Vertices[VertexIndex].Equals(Vertices[VertexIndex] * MeshScales[Index] * 0.01, Tolerance) ? 0 : 1;
            }
            TestEqual(*(Case + TEXT(": the vertices are scaled per axis and in meters")), Off, 0);
            TestEqual(*(Case + TEXT(": every triangle faces out")), CountOutwardTriangles(Mesh.Vertices, Mesh.Indices), 12);
            // With the winding of the input the scaled triangles would face in exactly when the scale mirrors.
            TestEqual(*(Case + TEXT(": the input winding on the scaled vertices")), CountOutwardTriangles(Mesh.Vertices, Indices), bMirrors[Index] ? 0 : 12);
            if (bMirrors[Index])
            {
                TestEqual(*(Case + TEXT(": the winding was swapped")), Mesh.Indices[1], Indices[2]);
            }
            else
            {
                TestEqual(*(Case + TEXT(": the winding was kept")), Mesh.Indices[1], Indices[1]);
            }
        }

        // Triangles that point outside the vertices are left out; with none left there is no shape.
        FUBotPhysicsShape Broken;
        const TArray<int32> MixedIndices = { 0, 1, 2, 0, 1, 99, -1, 1, 2 };
        const TArray<int32> BadIndices = { 0, 1, 99 };
        TestTrue(TEXT("Triangle mesh: bad triangles among good ones are left out"),
            UBot::Physics::Private::MakeTriangleMeshShape(Vertices, MixedIndices, FVector::OneVector, Broken) && Broken.Indices.Num() == 3);
        TestFalse(TEXT("Triangle mesh: nothing but bad triangles is no shape"),
            UBot::Physics::Private::MakeTriangleMeshShape(Vertices, BadIndices, FVector::OneVector, Broken));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSceneStaticAndKinematicTest, "UBotPhysics.Scene.StaticAndKinematic", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSceneStaticAndKinematicTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSceneTests;

    FScopedTestBackend Backend(StepSeconds);
    Backend.Factory->Capabilities = MakeSceneCapabilities(true, true);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }
    const UPhysicalMaterial* DefaultMaterial = GEngine != nullptr ? GEngine->DefaultPhysMaterial.Get() : nullptr;
    if (!TestNotNull(TEXT("The engine has a default physical material"), DefaultMaterial))
    {
        return false;
    }

    // A static box, a movable box with a material of its own and a body for the backend.
    UPhysicalMaterial* Material = NewObject<UPhysicalMaterial>(GetTransientPackage());
    Material->Friction = 0.25f;
    Material->Restitution = 0.5f;
    AActor* StaticActor = SpawnCollider(Scope, FVector(100.0, 200.0, 300.0), EComponentMobility::Static, FVector(25.0, 50.0, 75.0));
    AActor* MovableActor = SpawnCollider(Scope, FVector(-100.0, 0.0, 50.0), EComponentMobility::Movable, FVector(10.0, 20.0, 30.0), Material);
    AActor* BoundActor = SpawnCollider(Scope, FVector(500.0, 0.0, 0.0), EComponentMobility::Movable);
    UBoxComponent* StaticBox = GetBox(StaticActor);
    UBoxComponent* MovableBox = GetBox(MovableActor);
    UBoxComponent* BoundBox = GetBox(BoundActor);
    UUBotPhysicsTestBinding* Binding = AddBinding(*BoundActor, BackendName, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("The body binds"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }
    TestEqual(TEXT("The solver was sent nothing before the first frame"), Solver->SceneCalls.Num(), 0);

    // The first frame brings both boxes as added, the movable one also as a pose. The bound body is the solver's own.
    Scope.Tick(FrameSeconds);
    if (!TestEqual(TEXT("The first frame brings one update"), Solver->SceneCalls.Num(), 1))
    {
        return false;
    }
    const FUBotPhysicsSceneUpdate First = Solver->SceneCalls[0].Update;
    TestEqual(TEXT("Its time is the sim time of the frame"), First.SimTimeSeconds, Scope.Now(), 1.e-9);
    TestEqual(TEXT("Nothing is removed"), First.Removed.Num(), 0);
    if (!TestEqual(TEXT("The static box and the movable box are added, the bound body is not"), First.Added.Num(), 2))
    {
        return false;
    }
    const FUBotPhysicsSceneBody* StaticBody = FindAdded(First, StaticBox);
    const FUBotPhysicsSceneBody* MovableBody = FindAdded(First, MovableBox);
    if (!TestNotNull(TEXT("The static box is added"), StaticBody) || !TestNotNull(TEXT("The movable box is added"), MovableBody))
    {
        return false;
    }
    const int64 StaticId = StaticBody->Id;
    const int64 MovableId = MovableBody->Id;
    TestTrue(TEXT("The ids are positive and differ"), StaticId > 0 && MovableId > 0 && StaticId != MovableId);
    TestTrue(TEXT("The static box is a static body"), StaticBody->Kind == EUBotPhysicsSceneBodyKind::Static);
    TestTrue(TEXT("The movable box is a kinematic body"), MovableBody->Kind == EUBotPhysicsSceneBodyKind::Kinematic);
    TestTrue(TEXT("The static body is at its place, in meters"), StaticBody->WorldTransform.GetLocation().Equals(FVector(1.0, 2.0, 3.0), Tolerance));
    TestTrue(TEXT("Without rotation and without scale"), StaticBody->WorldTransform.GetRotation().Equals(FQuat::Identity, Tolerance)
        && StaticBody->WorldTransform.GetScale3D().Equals(FVector::OneVector, Tolerance));
    TestTrue(TEXT("The movable body is at its place, in meters"), MovableBody->WorldTransform.GetLocation().Equals(FVector(-1.0, 0.0, 0.5), Tolerance));
    if (TestEqual(TEXT("The static body has one shape"), StaticBody->Shapes.Num(), 1))
    {
        TestTrue(TEXT("A box"), StaticBody->Shapes[0].Type == EUBotPhysicsShapeType::Box);
        TestTrue(TEXT("With half extents in meters"), StaticBody->Shapes[0].Size.Equals(FVector(0.25, 0.5, 0.75), Tolerance));
        TestTrue(TEXT("In the body frame"), IsIdentity(StaticBody->Shapes[0].LocalTransform));
    }
    if (TestEqual(TEXT("The movable body has one shape"), MovableBody->Shapes.Num(), 1))
    {
        TestTrue(TEXT("With half extents in meters"), MovableBody->Shapes[0].Size.Equals(FVector(0.1, 0.2, 0.3), Tolerance));
    }
    TestTrue(TEXT("The movable body has the material it was given"), FMath::IsNearlyEqual(MovableBody->Friction, 0.25f) && FMath::IsNearlyEqual(MovableBody->Restitution, 0.5f));
    TestTrue(TEXT("The static body has the default material"),
        FMath::IsNearlyEqual(StaticBody->Friction, DefaultMaterial->Friction) && FMath::IsNearlyEqual(StaticBody->Restitution, DefaultMaterial->Restitution));
    if (TestEqual(TEXT("One kinematic pose: the movable box"), First.KinematicPoses.Num(), 1))
    {
        TestEqual(TEXT("It is the id of the movable box"), First.KinematicPoses[0].Id, MovableId);
        TestTrue(TEXT("At its place, in meters"), First.KinematicPoses[0].WorldTransform.GetLocation().Equals(FVector(-1.0, 0.0, 0.5), Tolerance));
    }
    TestEqual(TEXT("The solver knows one static body"), Solver->CountSceneBodies(EUBotPhysicsSceneBodyKind::Static), 1);
    TestEqual(TEXT("And one kinematic body"), Solver->CountSceneBodies(EUBotPhysicsSceneBodyKind::Kinematic), 1);
    TestEqual(TEXT("The bound body is not among them"), Solver->FindSceneBodyId(BoundBox), int64(INDEX_NONE));

    // Nothing changes: no additions, no removals, and the pose of the movable box again.
    Scope.Tick(FrameSeconds);
    if (!TestEqual(TEXT("Every frame brings an update while a kinematic body exists"), Solver->SceneCalls.Num(), 2))
    {
        return false;
    }
    const FUBotPhysicsSceneUpdate Second = Solver->SceneCalls[1].Update;
    TestTrue(TEXT("Nothing was added or removed"), Second.Added.IsEmpty() && Second.Removed.IsEmpty());
    TestTrue(TEXT("Only the pose of the movable box is sent"), Second.KinematicPoses.Num() == 1 && Second.KinematicPoses[0].Id == MovableId);

    // The movable box moves: the next update has its new pose only.
    MovableBox->SetWorldLocationAndRotation(FVector(0.0, 300.0, 200.0), FRotator(0.0, 90.0, 0.0));
    Scope.Tick(FrameSeconds);
    if (!TestEqual(TEXT("The frame after the move brings an update"), Solver->SceneCalls.Num(), 3))
    {
        return false;
    }
    const FUBotPhysicsSceneUpdate Third = Solver->SceneCalls[2].Update;
    TestTrue(TEXT("Nothing was added or removed by the move"), Third.Added.IsEmpty() && Third.Removed.IsEmpty());
    if (TestEqual(TEXT("One pose"), Third.KinematicPoses.Num(), 1))
    {
        TestEqual(TEXT("The movable box's"), Third.KinematicPoses[0].Id, MovableId);
        TestTrue(TEXT("At its new place, in meters"), Third.KinematicPoses[0].WorldTransform.GetLocation().Equals(FVector(0.0, 3.0, 2.0), Tolerance));
        TestTrue(TEXT("With its new rotation"), Third.KinematicPoses[0].WorldTransform.GetRotation().Equals(FRotator(0.0, 90.0, 0.0).Quaternion(), 1.e-5));
    }
    const FSceneBodyView* MovableView = Solver->SceneView.Find(MovableId);
    const FSceneBodyView* StaticView = Solver->SceneView.Find(StaticId);
    if (TestNotNull(TEXT("The solver still knows the movable body"), MovableView) && TestNotNull(TEXT("And the static one"), StaticView))
    {
        TestTrue(TEXT("It has the new pose of the movable body"), MovableView->WorldTransform.GetLocation().Equals(FVector(0.0, 3.0, 2.0), Tolerance));
        TestTrue(TEXT("The static body stayed"), StaticView->WorldTransform.GetLocation().Equals(FVector(1.0, 2.0, 3.0), Tolerance));
    }

    // The static box was sent once, as an addition, and never as a pose.
    int32 StaticAdditions = 0;
    int32 StaticPoses = 0;
    for (const FSceneCall& Call : Solver->SceneCalls)
    {
        StaticAdditions += Call.Update.Added.ContainsByPredicate([StaticId](const FUBotPhysicsSceneBody& Body) { return Body.Id == StaticId; }) ? 1 : 0;
        StaticPoses += FindPose(Call.Update, StaticId) != nullptr ? 1 : 0;
    }
    TestEqual(TEXT("The static body was added once"), StaticAdditions, 1);
    TestEqual(TEXT("And never sent again"), StaticPoses, 0);
    ExpectCleanProtocol(*this, TEXT("StaticAndKinematic"), *Solver);

    Binding->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSceneCapabilitiesTest, "UBotPhysics.Scene.Capabilities", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSceneCapabilitiesTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSceneTests;

    // One world per case, with a static box, a movable box and a body for the backend.
    constexpr int32 Frames = 5;
    const auto RunCase = [this](const TCHAR* Case, bool bStatic, bool bKinematic, int32 ExpectedUpdates, int32 ExpectedStaticBodies, int32 ExpectedKinematicBodies) -> bool
    {
        FScopedTestBackend Backend(StepSeconds);
        Backend.Factory->Capabilities = MakeSceneCapabilities(bStatic, bKinematic);
        FScopedPhysicsTestWorld Scope;
        UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
        if (!TestTrue(*FString::Printf(TEXT("%s: the test backend is registered"), Case), Backend.bRegistered)
            || !TestNotNull(*FString::Printf(TEXT("%s: the world has a physics subsystem"), Case), Subsystem))
        {
            return false;
        }
        SpawnCollider(Scope, FVector(100.0, 0.0, 0.0), EComponentMobility::Static);
        SpawnCollider(Scope, FVector(-100.0, 0.0, 0.0), EComponentMobility::Movable);
        AActor* BoundActor = SpawnCollider(Scope, FVector(500.0, 0.0, 0.0), EComponentMobility::Movable);
        UUBotPhysicsTestBinding* Binding = AddBinding(*BoundActor, BackendName, FVector(1.0, 0.0, 0.0));
        if (!TestTrue(*FString::Printf(TEXT("%s: the body binds"), Case), Binding->Bind()))
        {
            return false;
        }

        Scope.RunFrames(Frames);
        FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
        if (!TestNotNull(*FString::Printf(TEXT("%s: the solver exists"), Case), Solver))
        {
            return false;
        }
        TestTrue(*FString::Printf(TEXT("%s: the frames ran and the solver stepped"), Case), Solver->StepEndTimes.Num() > 0);
        TestEqual(*FString::Printf(TEXT("%s: number of updates"), Case), Solver->SceneCalls.Num(), ExpectedUpdates);
        TestEqual(*FString::Printf(TEXT("%s: static bodies the solver knows"), Case), Solver->CountSceneBodies(EUBotPhysicsSceneBodyKind::Static), ExpectedStaticBodies);
        TestEqual(*FString::Printf(TEXT("%s: kinematic bodies the solver knows"), Case), Solver->CountSceneBodies(EUBotPhysicsSceneBodyKind::Kinematic), ExpectedKinematicBodies);
        for (const FSceneCall& Call : Solver->SceneCalls)
        {
            if (!bKinematic)
            {
                TestTrue(*FString::Printf(TEXT("%s: no kinematic body is ever sent"), Case),
                    Call.Update.KinematicPoses.IsEmpty()
                    && !Call.Update.Added.ContainsByPredicate([](const FUBotPhysicsSceneBody& Body) { return Body.Kind == EUBotPhysicsSceneBodyKind::Kinematic; }));
            }
            if (!bStatic)
            {
                TestFalse(*FString::Printf(TEXT("%s: no static body is ever sent"), Case),
                    Call.Update.Added.ContainsByPredicate([](const FUBotPhysicsSceneBody& Body) { return Body.Kind == EUBotPhysicsSceneBodyKind::Static; }));
            }
        }
        ExpectCleanProtocol(*this, Case, *Solver);
        Binding->Unbind();
        return true;
    };

    RunCase(TEXT("Neither capability"), false, false, 0, 0, 0);
    // The static box is sent once; without moving bodies there is nothing to send afterwards.
    RunCase(TEXT("Static geometry only"), true, false, 1, 1, 0);
    // The poses of the movable box come every frame.
    RunCase(TEXT("Kinematic foreign bodies only"), false, true, Frames, 0, 1);

    // A solver without bodies gets nothing, however much changes; with a body again it catches up.
    {
        FScopedTestBackend Backend(StepSeconds);
        Backend.Factory->Capabilities = MakeSceneCapabilities(true, true);
        FScopedPhysicsTestWorld Scope;
        UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
        if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
        {
            return false;
        }
        SpawnCollider(Scope, FVector(100.0, 0.0, 0.0), EComponentMobility::Static);
        AActor* MovableActor = SpawnCollider(Scope, FVector(-100.0, 0.0, 0.0), EComponentMobility::Movable);
        AActor* BoundActor = SpawnCollider(Scope, FVector(500.0, 0.0, 0.0), EComponentMobility::Movable);
        UUBotPhysicsTestBinding* Binding = AddBinding(*BoundActor, BackendName, FVector(1.0, 0.0, 0.0));
        if (!TestTrue(TEXT("The body binds"), Binding->Bind()))
        {
            return false;
        }
        Scope.RunFrames(3);
        FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
        if (!TestNotNull(TEXT("The solver exists"), Solver))
        {
            return false;
        }
        const int32 UpdatesWhileBound = Solver->SceneCalls.Num();
        TestEqual(TEXT("A solver with a body is sent an update every frame"), UpdatesWhileBound, 3);

        Binding->Unbind();
        AActor* LateActor = SpawnCollider(Scope, FVector(300.0, 0.0, 0.0), EComponentMobility::Static);
        GetBox(MovableActor)->SetWorldLocation(FVector(0.0, 100.0, 0.0));
        Scope.RunFrames(5);
        TestNotNull(TEXT("The solver is still there"), FindTestSolver(*Subsystem));
        TestEqual(TEXT("A solver without bodies is sent no update"), Solver->SceneCalls.Num(), UpdatesWhileBound);

        if (!TestTrue(TEXT("The body binds again"), Binding->Bind()))
        {
            return false;
        }
        Scope.Tick(FrameSeconds);
        if (TestEqual(TEXT("With a body again the solver is sent an update"), Solver->SceneCalls.Num(), UpdatesWhileBound + 1))
        {
            const FUBotPhysicsSceneUpdate Resumed = Solver->SceneCalls.Last().Update;
            TestTrue(TEXT("It adds what was made meanwhile"), Resumed.Added.Num() == 1 && FindAdded(Resumed, GetBox(LateActor)) != nullptr);
            TestTrue(TEXT("And removes nothing"), Resumed.Removed.IsEmpty());
            const int64 MovableId = Solver->FindSceneBodyId(GetBox(MovableActor));
            const FUBotPhysicsScenePose* Pose = FindPose(Resumed, MovableId);
            TestTrue(TEXT("It has the pose the movable box has now"), Pose != nullptr && Pose->WorldTransform.GetLocation().Equals(FVector(0.0, 1.0, 0.0), Tolerance));
        }
        ExpectCleanProtocol(*this, TEXT("Solver without bodies"), *Solver);
        Binding->Unbind();
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSceneOwnBodiesExcludedTest, "UBotPhysics.Scene.OwnBodiesExcluded", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSceneOwnBodiesExcludedTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSceneTests;

    const FName SecondBackendName(TEXT("UBotPhysicsTestSecond"));
    FScopedTestBackend First(StepSeconds);
    FScopedTestBackend Second(StepSeconds, SecondBackendName, TEXT("UBotPhysicsTestsSecond"));
    First.Factory->Capabilities = MakeSceneCapabilities(true, true);
    Second.Factory->Capabilities = MakeSceneCapabilities(true, true);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("Both test backends are registered"), First.bRegistered && Second.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // X: a bound root of the first backend with a box attached to it. Z is another body of the first backend, Y one of
    // the second, so that both solvers hold bodies and are sent updates.
    AActor* ActorX = SpawnCollider(Scope, FVector(0.0, 0.0, 100.0), EComponentMobility::Movable);
    AActor* ActorY = SpawnCollider(Scope, FVector(1000.0, 0.0, 100.0), EComponentMobility::Movable);
    AActor* ActorZ = SpawnCollider(Scope, FVector(2000.0, 0.0, 100.0), EComponentMobility::Movable);
    UBoxComponent* RootX = GetBox(ActorX);
    UBoxComponent* RootY = GetBox(ActorY);
    UBoxComponent* RootZ = GetBox(ActorZ);
    UBoxComponent* ChildX = NewObject<UBoxComponent>(ActorX, TEXT("Child"));
    ChildX->SetBoxExtent(FVector(10.0), false);
    ChildX->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
    ChildX->SetupAttachment(RootX);
    ChildX->SetRelativeLocation(FVector(0.0, 100.0, 0.0));
    ChildX->RegisterComponent();

    UUBotPhysicsTestBinding* BindingX = AddBinding(*ActorX, BackendName, FVector(1.0, 0.0, 0.0));
    UUBotPhysicsTestBinding* BindingY = AddBinding(*ActorY, SecondBackendName, FVector(0.0, 1.0, 0.0));
    UUBotPhysicsTestBinding* BindingZ = AddBinding(*ActorZ, BackendName, FVector(0.0, 0.0, 1.0));
    if (!TestTrue(TEXT("X binds to the first backend"), BindingX->Bind()) || !TestTrue(TEXT("Y binds to the second"), BindingY->Bind())
        || !TestTrue(TEXT("Z binds to the first"), BindingZ->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* SolverA = FindTestSolver(*Subsystem);
    FUBotPhysicsTestSolver* SolverB = FindTestSolver(*Subsystem, SecondBackendName);
    if (!TestNotNull(TEXT("The first solver exists"), SolverA) || !TestNotNull(TEXT("The second solver exists"), SolverB))
    {
        return false;
    }

    // A has neither X nor the box on it, nor Z; it sees Y. B sees X, the box and Z as moving bodies, and not Y.
    Scope.Tick(FrameSeconds);
    TestEqual(TEXT("A sees one body"), SolverA->SceneView.Num(), 1);
    TestEqual(TEXT("It is not X"), SolverA->FindSceneBodyId(RootX), int64(INDEX_NONE));
    TestEqual(TEXT("Nor the box attached to X"), SolverA->FindSceneBodyId(ChildX), int64(INDEX_NONE));
    TestEqual(TEXT("Nor Z, another body of A"), SolverA->FindSceneBodyId(RootZ), int64(INDEX_NONE));
    const int64 IdYInA = SolverA->FindSceneBodyId(RootY);
    if (TestNotEqual(TEXT("It is Y"), IdYInA, int64(INDEX_NONE)))
    {
        TestTrue(TEXT("Y is a kinematic body"), SolverA->SceneView[IdYInA].Kind == EUBotPhysicsSceneBodyKind::Kinematic);
    }
    TestEqual(TEXT("B sees three bodies"), SolverB->SceneView.Num(), 3);
    TestEqual(TEXT("Not Y, its own body"), SolverB->FindSceneBodyId(RootY), int64(INDEX_NONE));
    const int64 IdXInB = SolverB->FindSceneBodyId(RootX);
    const int64 IdChildInB = SolverB->FindSceneBodyId(ChildX);
    const int64 IdZInB = SolverB->FindSceneBodyId(RootZ);
    if (!TestNotEqual(TEXT("But X"), IdXInB, int64(INDEX_NONE)) || !TestNotEqual(TEXT("And the box on X"), IdChildInB, int64(INDEX_NONE))
        || !TestNotEqual(TEXT("And Z"), IdZInB, int64(INDEX_NONE)))
    {
        return false;
    }
    TestTrue(TEXT("All three are kinematic bodies"), SolverB->CountSceneBodies(EUBotPhysicsSceneBodyKind::Kinematic) == 3);

    // X goes back to Chaos: A's next update adds X and the box on it, under the ids B knows them by. B's ids stay.
    BindingX->Unbind();
    Scope.Tick(FrameSeconds);
    {
        const FUBotPhysicsSceneUpdate Update = SolverA->SceneCalls.Last().Update;
        TestEqual(TEXT("A is sent X and the box on it"), Update.Added.Num(), 2);
        TestTrue(TEXT("As kinematic bodies"), Update.Added.Num() == 2 && Update.Added[0].Kind == EUBotPhysicsSceneBodyKind::Kinematic
            && Update.Added[1].Kind == EUBotPhysicsSceneBodyKind::Kinematic);
        TestTrue(TEXT("And removes nothing"), Update.Removed.IsEmpty());
        TestEqual(TEXT("X has the id B knows it by"), SolverA->FindSceneBodyId(RootX), IdXInB);
        TestEqual(TEXT("And the box"), SolverA->FindSceneBodyId(ChildX), IdChildInB);
        TestEqual(TEXT("A sees Y as before"), SolverA->FindSceneBodyId(RootY), IdYInA);
    }
    {
        const FUBotPhysicsSceneUpdate Update = SolverB->SceneCalls.Last().Update;
        TestTrue(TEXT("B is sent no addition and no removal"), Update.Added.IsEmpty() && Update.Removed.IsEmpty());
        TestEqual(TEXT("B still knows X by its id"), SolverB->FindSceneBodyId(RootX), IdXInB);
        TestEqual(TEXT("And the box"), SolverB->FindSceneBodyId(ChildX), IdChildInB);
        TestEqual(TEXT("And Z"), SolverB->FindSceneBodyId(RootZ), IdZInB);
    }

    // X is bound again: A drops the two.
    TestTrue(TEXT("X binds again"), BindingX->Bind());
    Scope.Tick(FrameSeconds);
    {
        const FUBotPhysicsSceneUpdate Update = SolverA->SceneCalls.Last().Update;
        TestEqual(TEXT("A is told that X and the box left"), Update.Removed.Num(), 2);
        TestTrue(TEXT("Under their ids"), Update.Removed.Contains(IdXInB) && Update.Removed.Contains(IdChildInB));
        TestTrue(TEXT("And that nothing came"), Update.Added.IsEmpty());
        TestEqual(TEXT("B's ids are unchanged"), SolverB->FindSceneBodyId(RootX), IdXInB);
    }
    ExpectCleanProtocol(*this, TEXT("A"), *SolverA);
    ExpectCleanProtocol(*this, TEXT("B"), *SolverB);

    BindingX->Unbind();
    BindingY->Unbind();
    BindingZ->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSceneChangesTest, "UBotPhysics.Scene.Changes", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSceneChangesTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSceneTests;

    FScopedTestBackend Backend(StepSeconds);
    Backend.Factory->Capabilities = MakeSceneCapabilities(true, true);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("The test backend is registered"), Backend.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // A body for the backend, and the components that will change: all movable boxes with physics collision unless noted.
    AActor* BoundActor = SpawnCollider(Scope, FVector(5000.0, 0.0, 0.0), EComponentMobility::Movable);
    AActor* DestroyedActor = SpawnCollider(Scope, FVector(0.0, 0.0, 0.0), EComponentMobility::Movable);
    AActor* CollisionActor = SpawnCollider(Scope, FVector(500.0, 0.0, 0.0), EComponentMobility::Movable);
    AActor* TaggedActor = SpawnCollider(Scope, FVector(1000.0, 0.0, 0.0), EComponentMobility::Static);
    AActor* ComponentTaggedActor = SpawnCollider(Scope, FVector(1500.0, 0.0, 0.0), EComponentMobility::Movable);
    AActor* ScaledActor = SpawnCollider(Scope, FVector(2000.0, 0.0, 0.0), EComponentMobility::Movable);
    AActor* MobilityActor = SpawnCollider(Scope, FVector(2500.0, 0.0, 0.0), EComponentMobility::Static);
    AActor* QueryOnlyActor = SpawnCollider(Scope, FVector(3000.0, 0.0, 0.0), EComponentMobility::Movable);
    UBoxComponent* DestroyedBox = GetBox(DestroyedActor);
    UBoxComponent* CollisionBox = GetBox(CollisionActor);
    UBoxComponent* TaggedBox = GetBox(TaggedActor);
    UBoxComponent* ComponentTaggedBox = GetBox(ComponentTaggedActor);
    UBoxComponent* ScaledBox = GetBox(ScaledActor);
    UBoxComponent* MobilityBox = GetBox(MobilityActor);
    UBoxComponent* QueryOnlyBox = GetBox(QueryOnlyActor);
    QueryOnlyBox->SetCollisionEnabled(ECollisionEnabled::QueryOnly);

    UUBotPhysicsTestBinding* Binding = AddBinding(*BoundActor, BackendName, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("The body binds"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The solver exists"), Solver))
    {
        return false;
    }

    // The next frame, and the update it brought.
    const auto NextUpdate = [&Scope, Solver]() -> FUBotPhysicsSceneUpdate
    {
        const int32 Before = Solver->SceneCalls.Num();
        Scope.Tick(FrameSeconds);
        return Solver->SceneCalls.Num() > Before ? Solver->SceneCalls.Last().Update : FUBotPhysicsSceneUpdate();
    };

    FUBotPhysicsSceneUpdate Update = NextUpdate();
    TestEqual(TEXT("The first update adds the six components with physics collision"), Update.Added.Num(), 6);
    TestEqual(TEXT("The one with query collision only is not among them"), Solver->FindSceneBodyId(QueryOnlyBox), int64(INDEX_NONE));
    const int64 DestroyedId = Solver->FindSceneBodyId(DestroyedBox);
    const int64 CollisionId = Solver->FindSceneBodyId(CollisionBox);
    const int64 TaggedId = Solver->FindSceneBodyId(TaggedBox);
    const int64 ComponentTaggedId = Solver->FindSceneBodyId(ComponentTaggedBox);
    const int64 ScaledId = Solver->FindSceneBodyId(ScaledBox);
    const int64 MobilityId = Solver->FindSceneBodyId(MobilityBox);

    FUBotPhysicsSceneStats Baseline;
    TestTrue(TEXT("The backend has scene stats"), Subsystem->GetSceneStats(BackendName, Baseline));

    // Destroying a component removes it.
    DestroyedBox->DestroyComponent();
    Update = NextUpdate();
    TestTrue(TEXT("A destroyed component is removed"), Update.Removed.Num() == 1 && Update.Removed[0] == DestroyedId);
    TestTrue(TEXT("Nothing is added"), Update.Added.IsEmpty());
    TestFalse(TEXT("The solver forgot it"), Solver->SceneView.Contains(DestroyedId));

    // Collision off removes it, collision on adds it again under a new id.
    CollisionBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Update = NextUpdate();
    TestTrue(TEXT("A component without collision is removed"), Update.Removed.Num() == 1 && Update.Removed[0] == CollisionId);
    CollisionBox->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    Update = NextUpdate();
    if (TestEqual(TEXT("With collision again it is added"), Update.Added.Num(), 1))
    {
        TestTrue(TEXT("It is the same component"), Update.Added[0].Component.Get() == CollisionBox);
        TestTrue(TEXT("Under a new id"), Update.Added[0].Id != CollisionId && Update.Added[0].Id > 0);
    }
    TestTrue(TEXT("Nothing is removed by that"), Update.Removed.IsEmpty());

    // The tag keeps a component out, on the actor or on the component itself.
    TaggedActor->Tags.Add(UBot::Physics::NoSceneTag);
    Update = NextUpdate();
    TestTrue(TEXT("A component of a tagged actor is removed"), Update.Removed.Num() == 1 && Update.Removed[0] == TaggedId);
    ComponentTaggedBox->ComponentTags.Add(UBot::Physics::NoSceneTag);
    Update = NextUpdate();
    TestTrue(TEXT("A tagged component is removed"), Update.Removed.Num() == 1 && Update.Removed[0] == ComponentTaggedId);
    TaggedActor->Tags.Remove(UBot::Physics::NoSceneTag);
    Update = NextUpdate();
    if (TestEqual(TEXT("Without the tag the component is added again"), Update.Added.Num(), 1))
    {
        TestTrue(TEXT("Under a new id"), Update.Added[0].Id != TaggedId && Update.Added[0].Component.Get() == TaggedBox);
    }

    // A changed world scale ends the export and starts another: removed and added in one update. The pose stays unscaled.
    ScaledBox->SetWorldScale3D(FVector(2.0));
    Update = NextUpdate();
    TestTrue(TEXT("A rescaled component leaves under its old id"), Update.Removed.Num() == 1 && Update.Removed[0] == ScaledId);
    if (TestEqual(TEXT("And comes back"), Update.Added.Num(), 1))
    {
        TestTrue(TEXT("Under a new id"), Update.Added[0].Id != ScaledId && Update.Added[0].Component.Get() == ScaledBox);
        if (TestEqual(TEXT("With a shape"), Update.Added[0].Shapes.Num(), 1))
        {
            TestTrue(TEXT("Twice as large"), Update.Added[0].Shapes[0].Size.Equals(FVector(0.5), Tolerance));
        }
        TestTrue(TEXT("At the same place without scale"), Update.Added[0].WorldTransform.GetLocation().Equals(FVector(20.0, 0.0, 0.0), Tolerance)
            && Update.Added[0].WorldTransform.GetScale3D().Equals(FVector::OneVector, Tolerance));
    }

    // A changed mobility does the same, and changes the kind.
    MobilityBox->SetMobility(EComponentMobility::Movable);
    Update = NextUpdate();
    TestTrue(TEXT("A component that became movable leaves under its old id"), Update.Removed.Num() == 1 && Update.Removed[0] == MobilityId);
    int64 MovableMobilityId = INDEX_NONE;
    if (TestEqual(TEXT("And comes back"), Update.Added.Num(), 1))
    {
        MovableMobilityId = Update.Added[0].Id;
        TestTrue(TEXT("Under a new id, as a kinematic body"), MovableMobilityId != MobilityId && Update.Added[0].Kind == EUBotPhysicsSceneBodyKind::Kinematic);
    }
    MobilityBox->SetMobility(EComponentMobility::Static);
    Update = NextUpdate();
    TestTrue(TEXT("And back to static"), Update.Removed.Num() == 1 && Update.Removed[0] == MovableMobilityId);
    if (TestEqual(TEXT("It comes back as a static body"), Update.Added.Num(), 1))
    {
        TestTrue(TEXT("Under a third id"), Update.Added[0].Id != MobilityId && Update.Added[0].Id != MovableMobilityId && Update.Added[0].Kind == EUBotPhysicsSceneBodyKind::Static);
    }

    // Components with physics collision that cannot be exported are counted: a skinned and an instanced one.
    {
        AActor* SkinnedActor = SpawnBlockingActor<USkeletalMeshComponent>(Scope);
        AActor* InstancedActor = SpawnBlockingActor<UInstancedStaticMeshComponent>(Scope);
        TestTrue(TEXT("Both have physics collision"), SkinnedActor->GetRootComponent()->GetCollisionEnabled() == ECollisionEnabled::QueryAndPhysics
            && InstancedActor->GetRootComponent()->GetCollisionEnabled() == ECollisionEnabled::QueryAndPhysics);
        Update = NextUpdate();
        TestTrue(TEXT("Neither is sent"), Update.Added.IsEmpty() && Update.Removed.IsEmpty());
        FUBotPhysicsSceneStats Stats;
        if (TestTrue(TEXT("The backend has scene stats"), Subsystem->GetSceneStats(BackendName, Stats)))
        {
            TestEqual(TEXT("Both are counted as skipped"), Stats.SkippedComponents, Baseline.SkippedComponents + 2);
        }
    }
    ExpectCleanProtocol(*this, TEXT("Changes"), *Solver);

    Binding->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSceneUpdateBeforeStepsTest, "UBotPhysics.Scene.UpdateBeforeSteps", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSceneUpdateBeforeStepsTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSceneTests;

    // Runs Frames frames of FrameDelta, and checks each: at most one update, before the steps of the frame, with the sim time of the frame.
    const auto RunFrames = [this](const TCHAR* Case, double SolverStepSeconds, int32 Frames, float FrameDelta, int32& OutFramesWithSteps, int32& OutFramesWithoutSteps) -> bool
    {
        FScopedTestBackend Backend(SolverStepSeconds);
        Backend.Factory->Capabilities = MakeSceneCapabilities(true, true);
        FScopedPhysicsTestWorld Scope;
        UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
        if (!TestTrue(*FString::Printf(TEXT("%s: the test backend is registered"), Case), Backend.bRegistered)
            || !TestNotNull(*FString::Printf(TEXT("%s: the world has a physics subsystem"), Case), Subsystem))
        {
            return false;
        }
        SpawnCollider(Scope, FVector(100.0, 0.0, 0.0), EComponentMobility::Static);
        SpawnCollider(Scope, FVector(-100.0, 0.0, 0.0), EComponentMobility::Movable);
        AActor* BoundActor = SpawnCollider(Scope, FVector(500.0, 0.0, 0.0), EComponentMobility::Movable);
        UUBotPhysicsTestBinding* Binding = AddBinding(*BoundActor, BackendName, FVector(1.0, 0.0, 0.0));
        if (!TestTrue(*FString::Printf(TEXT("%s: the body binds"), Case), Binding->Bind()))
        {
            return false;
        }
        FUBotPhysicsTestSolver* Solver = FindTestSolver(*Subsystem);
        if (!TestNotNull(*FString::Printf(TEXT("%s: the solver exists"), Case), Solver))
        {
            return false;
        }

        OutFramesWithSteps = 0;
        OutFramesWithoutSteps = 0;
        int32 BadFrames = 0;
        for (int32 Frame = 0; Frame < Frames; ++Frame)
        {
            Solver->CallLog.Reset();
            const int32 UpdatesBefore = Solver->SceneCalls.Num();
            const int32 StepsBefore = Solver->StepEndTimes.Num();
            Scope.Tick(FrameDelta);

            int32 Updates = 0;
            int32 FirstStepIndex = INDEX_NONE;
            int32 UpdateIndex = INDEX_NONE;
            for (int32 CallIndex = 0; CallIndex < Solver->CallLog.Num(); ++CallIndex)
            {
                if (Solver->CallLog[CallIndex] == ESolverCall::UpdateScene)
                {
                    ++Updates;
                    UpdateIndex = CallIndex;
                }
                else if (FirstStepIndex == INDEX_NONE)
                {
                    FirstStepIndex = CallIndex;
                }
            }
            const int32 Steps = Solver->StepEndTimes.Num() - StepsBefore;
            const bool bUpdateOnce = Updates == 1 && Solver->SceneCalls.Num() == UpdatesBefore + 1;
            const bool bBeforeSteps = FirstStepIndex == INDEX_NONE || UpdateIndex < FirstStepIndex;
            bool bSimTime = false;
            bool bStepsAfter = false;
            if (bUpdateOnce)
            {
                bSimTime = FMath::IsNearlyEqual(Solver->SceneCalls.Last().Update.SimTimeSeconds, Scope.Now(), 1.e-9);
                bStepsAfter = Solver->SceneCalls.Last().StepsBefore == StepsBefore;
            }
            BadFrames += (bUpdateOnce && bBeforeSteps && bSimTime && bStepsAfter) ? 0 : 1;
            (Steps > 0 ? OutFramesWithSteps : OutFramesWithoutSteps) += 1;
        }
        TestEqual(*FString::Printf(TEXT("%s: every frame has exactly one update, before its steps, with the sim time of the frame"), Case), BadFrames, 0);
        ExpectCleanProtocol(*this, Case, *Solver);
        Binding->Unbind();
        return true;
    };

    // A step of 100 ms against frames of 16.7 ms: most frames take no step, and the update comes all the same, also in the
    // first frame, which starts the solver's clock.
    int32 WithSteps = 0;
    int32 WithoutSteps = 0;
    if (RunFrames(TEXT("Slow solver"), 0.1, 24, FrameSeconds, WithSteps, WithoutSteps))
    {
        TestTrue(TEXT("Slow solver: some frames ran steps"), WithSteps >= 2);
        TestTrue(TEXT("Slow solver: most did not"), WithoutSteps >= 10);
    }

    // A step of 10 ms against frames of 50 ms: several steps in a frame, one update before them.
    if (RunFrames(TEXT("Fast solver"), 0.01, 6, 0.05f, WithSteps, WithoutSteps))
    {
        TestTrue(TEXT("Fast solver: all frames after the first ran steps"), WithSteps >= 4);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSceneReleasedBackendTest, "UBotPhysics.Scene.ReleasedBackend", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSceneReleasedBackendTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSceneTests;

    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    TUniquePtr<FScopedTestBackend> First = MakeUnique<FScopedTestBackend>(StepSeconds);
    if (!TestTrue(TEXT("The test backend is registered"), First->bRegistered))
    {
        return false;
    }
    First->Factory->Capabilities = MakeSceneCapabilities(true, true);

    AActor* StaticActor = SpawnCollider(Scope, FVector(100.0, 0.0, 0.0), EComponentMobility::Static);
    AActor* MovableActor = SpawnCollider(Scope, FVector(-100.0, 0.0, 0.0), EComponentMobility::Movable);
    AActor* BoundActor = SpawnCollider(Scope, FVector(500.0, 0.0, 0.0), EComponentMobility::Movable);
    UBoxComponent* StaticBox = GetBox(StaticActor);
    UBoxComponent* MovableBox = GetBox(MovableActor);
    UUBotPhysicsTestBinding* Binding = AddBinding(*BoundActor, BackendName, FVector(1.0, 0.0, 0.0));
    if (!TestTrue(TEXT("The body binds"), Binding->Bind()))
    {
        return false;
    }
    Scope.RunFrames(3);
    const TWeakPtr<IUBotPhysicsSolver> FirstSolver = First->Factory->LastSolver;
    const FUBotPhysicsTestSolver* SolverBefore = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("The first solver exists"), SolverBefore))
    {
        return false;
    }
    TestEqual(TEXT("The first solver knows both boxes"), SolverBefore->SceneView.Num(), 2);
    FUBotPhysicsSceneStats Stats;
    TestTrue(TEXT("The backend has scene stats while it has a solver"), Subsystem->GetSceneStats(BackendName, Stats));
    TestTrue(TEXT("One static body and one kinematic body"), Stats.StaticBodies == 1 && Stats.KinematicBodies == 1);

    // The backend goes away: its solver and with it what the solver knew.
    First.Reset();
    TestTrue(TEXT("The solver is gone"), Subsystem->FindSolver(BackendName) == nullptr && !FirstSolver.IsValid());
    TestFalse(TEXT("So are its stats"), Subsystem->GetSceneStats(BackendName, Stats));
    TestTrue(TEXT("Which are zero"), Stats.StaticBodies == 0 && Stats.KinematicBodies == 0 && Stats.Shapes == 0 && Stats.SkippedComponents == 0);

    // It comes back as a new factory: the next bind makes a new solver, and its first update holds the whole scene.
    FScopedTestBackend Second(StepSeconds);
    if (!TestTrue(TEXT("The backend registers again"), Second.bRegistered))
    {
        return false;
    }
    Second.Factory->Capabilities = MakeSceneCapabilities(true, true);
    if (!TestTrue(TEXT("The body binds again"), Binding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* SolverAfter = FindTestSolver(*Subsystem);
    if (!TestNotNull(TEXT("A new solver exists"), SolverAfter))
    {
        return false;
    }
    TestTrue(TEXT("It starts out knowing nothing of the scene"), SolverAfter->SceneCalls.IsEmpty() && SolverAfter->SceneView.IsEmpty());
    Scope.Tick(FrameSeconds);
    if (TestEqual(TEXT("Its first frame brings an update"), SolverAfter->SceneCalls.Num(), 1))
    {
        const FUBotPhysicsSceneUpdate Update = SolverAfter->SceneCalls[0].Update;
        TestTrue(TEXT("Which adds both boxes"), Update.Added.Num() == 2 && FindAdded(Update, StaticBox) != nullptr && FindAdded(Update, MovableBox) != nullptr);
        TestTrue(TEXT("And removes nothing"), Update.Removed.IsEmpty());
    }
    TestTrue(TEXT("The backend has scene stats again"), Subsystem->GetSceneStats(BackendName, Stats));
    TestTrue(TEXT("One static body and one kinematic body"), Stats.StaticBodies == 1 && Stats.KinematicBodies == 1);
    ExpectCleanProtocol(*this, TEXT("ReleasedBackend"), *SolverAfter);

    Binding->Unbind();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsSceneStatsTest, "UBotPhysics.Scene.Stats", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsSceneStatsTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsSceneTests;

    const FName StaticOnlyName(TEXT("UBotPhysicsTestStaticOnly"));
    FScopedTestBackend Full(StepSeconds);
    FScopedTestBackend StaticOnly(StepSeconds, StaticOnlyName, TEXT("UBotPhysicsTestsStaticOnly"));
    Full.Factory->Capabilities = MakeSceneCapabilities(true, true);
    StaticOnly.Factory->Capabilities = MakeSceneCapabilities(true, false);
    FScopedPhysicsTestWorld Scope;
    UUBotPhysicsSubsystem* Subsystem = Scope.GetSubsystem();
    if (!TestTrue(TEXT("Both test backends are registered"), Full.bRegistered && StaticOnly.bRegistered) || !TestNotNull(TEXT("The world has a physics subsystem"), Subsystem))
    {
        return false;
    }

    // Two static boxes, two movable boxes and a body for each backend.
    SpawnCollider(Scope, FVector(100.0, 0.0, 0.0), EComponentMobility::Static);
    SpawnCollider(Scope, FVector(200.0, 0.0, 0.0), EComponentMobility::Static);
    SpawnCollider(Scope, FVector(-100.0, 0.0, 0.0), EComponentMobility::Movable);
    SpawnCollider(Scope, FVector(-200.0, 0.0, 0.0), EComponentMobility::Movable);
    AActor* FullActor = SpawnCollider(Scope, FVector(500.0, 0.0, 0.0), EComponentMobility::Movable);
    AActor* StaticOnlyActor = SpawnCollider(Scope, FVector(600.0, 0.0, 0.0), EComponentMobility::Movable);

    // Before a solver exists there are no stats, in C++ and in Blueprint.
    FUBotPhysicsSceneStats Stats;
    FUBotPhysicsSceneStats NodeStats;
    NodeStats.StaticBodies = 7;
    TestFalse(TEXT("No solver: no stats"), Subsystem->GetSceneStats(BackendName, Stats));
    TestFalse(TEXT("No solver: no stats from the node either"), UUBotPhysicsLibrary::GetPhysicsSceneStats(Scope.Get(), BackendName, NodeStats));
    TestTrue(TEXT("The node leaves zeros"), NodeStats.StaticBodies == 0 && NodeStats.KinematicBodies == 0 && NodeStats.Shapes == 0 && NodeStats.SkippedComponents == 0);

    UUBotPhysicsTestBinding* FullBinding = AddBinding(*FullActor, BackendName, FVector(1.0, 0.0, 0.0));
    UUBotPhysicsTestBinding* StaticOnlyBinding = AddBinding(*StaticOnlyActor, StaticOnlyName, FVector(0.0, 1.0, 0.0));
    if (!TestTrue(TEXT("The first body binds"), FullBinding->Bind()) || !TestTrue(TEXT("The second body binds"), StaticOnlyBinding->Bind()))
    {
        return false;
    }
    FUBotPhysicsTestSolver* FullSolver = FindTestSolver(*Subsystem);
    FUBotPhysicsTestSolver* StaticOnlySolver = FindTestSolver(*Subsystem, StaticOnlyName);
    if (!TestNotNull(TEXT("The first solver exists"), FullSolver) || !TestNotNull(TEXT("The second solver exists"), StaticOnlySolver))
    {
        return false;
    }
    Scope.RunFrames(3);

    // The first backend sees the static boxes, the movable ones and the second backend's body.
    if (TestTrue(TEXT("The first backend has scene stats"), Subsystem->GetSceneStats(BackendName, Stats)))
    {
        TestEqual(TEXT("Static bodies"), Stats.StaticBodies, 2);
        TestEqual(TEXT("Kinematic bodies: two movable boxes and the other backend's body"), Stats.KinematicBodies, 3);
        TestEqual(TEXT("One shape each"), Stats.Shapes, 5);
        TestEqual(TEXT("Static bodies the solver knows"), Stats.StaticBodies, FullSolver->CountSceneBodies(EUBotPhysicsSceneBodyKind::Static));
        TestEqual(TEXT("Kinematic bodies the solver knows"), Stats.KinematicBodies, FullSolver->CountSceneBodies(EUBotPhysicsSceneBodyKind::Kinematic));
        TestEqual(TEXT("Shapes the solver knows"), Stats.Shapes, FullSolver->CountSceneShapes());
    }
    // The second only asks for static geometry.
    FUBotPhysicsSceneStats StaticOnlyStats;
    if (TestTrue(TEXT("The second backend has scene stats"), Subsystem->GetSceneStats(StaticOnlyName, StaticOnlyStats)))
    {
        TestTrue(TEXT("Two static bodies, no kinematic ones, two shapes"), StaticOnlyStats.StaticBodies == 2 && StaticOnlyStats.KinematicBodies == 0 && StaticOnlyStats.Shapes == 2);
        TestEqual(TEXT("Shapes the solver knows"), StaticOnlyStats.Shapes, StaticOnlySolver->CountSceneShapes());
    }
    TestFalse(TEXT("A backend that does not exist has no stats"), Subsystem->GetSceneStats(TEXT("NoSuchBackend"), NodeStats));

    // Components that cannot be exported are counted, once per scan.
    const int32 BaselineSkipped = Stats.SkippedComponents;
    SpawnBlockingActor<USkeletalMeshComponent>(Scope);
    SpawnBlockingActor<UInstancedStaticMeshComponent>(Scope);
    Scope.RunFrames(2);
    Subsystem->GetSceneStats(BackendName, Stats);
    TestEqual(TEXT("Two components with physics collision could not be exported"), Stats.SkippedComponents, BaselineSkipped + 2);

    // The Blueprint node says the same, for either backend.
    if (TestTrue(TEXT("The node has stats"), UUBotPhysicsLibrary::GetPhysicsSceneStats(Scope.Get(), BackendName, NodeStats)))
    {
        TestTrue(TEXT("The node and the subsystem agree"), NodeStats.StaticBodies == Stats.StaticBodies && NodeStats.KinematicBodies == Stats.KinematicBodies
            && NodeStats.Shapes == Stats.Shapes && NodeStats.SkippedComponents == Stats.SkippedComponents);
    }
    FUBotPhysicsSceneStats StaticOnlyNodeStats;
    Subsystem->GetSceneStats(StaticOnlyName, StaticOnlyStats);
    if (TestTrue(TEXT("The node has stats for the second backend"), UUBotPhysicsLibrary::GetPhysicsSceneStats(Scope.Get(), StaticOnlyName, StaticOnlyNodeStats)))
    {
        TestTrue(TEXT("The node and the subsystem agree for it as well"), StaticOnlyNodeStats.StaticBodies == StaticOnlyStats.StaticBodies
            && StaticOnlyNodeStats.KinematicBodies == StaticOnlyStats.KinematicBodies && StaticOnlyNodeStats.Shapes == StaticOnlyStats.Shapes
            && StaticOnlyNodeStats.SkippedComponents == StaticOnlyStats.SkippedComponents);
    }
    TestEqual(TEXT("The scan is shared: both count the same skipped components"), StaticOnlyStats.SkippedComponents, Stats.SkippedComponents);

    ExpectCleanProtocol(*this, TEXT("Stats, first backend"), *FullSolver);
    ExpectCleanProtocol(*this, TEXT("Stats, second backend"), *StaticOnlySolver);
    FullBinding->Unbind();
    StaticOnlyBinding->Unbind();
    return true;
}

#endif
