#include "UBotPhysicsScene.h"

#include "Chaos/TriangleMeshImplicitObject.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkinnedMeshComponent.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "PhysicsEngine/BodySetup.h"
#include "UBotPhysicsSceneShapes.h"
#include "UBotUnits.h"

namespace UBot::Physics::Private
{
    namespace
    {
        /** Number of components of Scale3D that are negative. */
        int32 CountNegativeComponents(const FVector& Scale3D)
        {
            return (Scale3D.X < 0.0 ? 1 : 0) + (Scale3D.Y < 0.0 ? 1 : 0) + (Scale3D.Z < 0.0 ? 1 : 0);
        }

        /** Positions and triangles of a Chaos triangle mesh, in the frame of its body setup (centimeters). */
        void ReadChaosTriangleMesh(const Chaos::FTriangleMeshImplicitObject& TriangleMesh, TArray<FVector>& OutVertices, TArray<int32>& OutIndices)
        {
            const Chaos::FTriangleMeshImplicitObject::ParticlesType& Particles = TriangleMesh.Particles();
            const int32 VertexCount = static_cast<int32>(Particles.Size());
            OutVertices.Reserve(VertexCount);
            for (int32 VertexIndex = 0; VertexIndex < VertexCount; ++VertexIndex)
            {
                const auto& Position = Particles.GetX(VertexIndex);
                OutVertices.Add(FVector(Position.X, Position.Y, Position.Z));
            }

            // The triangle indices are 16 or 32 bits wide, depending on the size of the mesh.
            const auto AppendTriangles = [&OutIndices](const auto& Triangles)
            {
                OutIndices.Reserve(OutIndices.Num() + Triangles.Num() * 3);
                for (const auto& Triangle : Triangles)
                {
                    OutIndices.Add(static_cast<int32>(Triangle[0]));
                    OutIndices.Add(static_cast<int32>(Triangle[1]));
                    OutIndices.Add(static_cast<int32>(Triangle[2]));
                }
            };
            const Chaos::FTrimeshIndexBuffer& Elements = TriangleMesh.Elements();
            if (Elements.RequiresLargeIndices())
            {
                AppendTriangles(Elements.GetLargeIndexBuffer());
            }
            else
            {
                AppendTriangles(Elements.GetSmallIndexBuffer());
            }
        }
    }

    bool MakeTriangleMeshShape(TConstArrayView<FVector> VerticesCentimeters, TConstArrayView<int32> Indices, const FVector& Scale3D, FUBotPhysicsShape& OutShape)
    {
        OutShape = FUBotPhysicsShape();
        OutShape.Type = EUBotPhysicsShapeType::TriangleMesh;

        // The triangles are inside out when the scale mirrors an odd number of axes.
        const bool bMirrored = (CountNegativeComponents(Scale3D) % 2) == 1;
        const int32 VertexCount = VerticesCentimeters.Num();

        OutShape.Vertices.Reserve(VertexCount);
        for (const FVector& Vertex : VerticesCentimeters)
        {
            OutShape.Vertices.Add(UBot::Units::CentimetersToMeters(Vertex * Scale3D));
        }

        const int32 TriangleCount = Indices.Num() / 3;
        OutShape.Indices.Reserve(TriangleCount * 3);
        for (int32 TriangleIndex = 0; TriangleIndex < TriangleCount; ++TriangleIndex)
        {
            const int32 A = Indices[TriangleIndex * 3];
            const int32 B = Indices[TriangleIndex * 3 + 1];
            const int32 C = Indices[TriangleIndex * 3 + 2];
            if (A < 0 || A >= VertexCount || B < 0 || B >= VertexCount || C < 0 || C >= VertexCount)
            {
                continue;
            }
            OutShape.Indices.Add(A);
            OutShape.Indices.Add(bMirrored ? C : B);
            OutShape.Indices.Add(bMirrored ? B : C);
        }
        return !OutShape.Indices.IsEmpty();
    }
}

namespace UBot::Physics
{
    bool GetCollisionShapes(const UPrimitiveComponent& Component, TArray<FUBotPhysicsShape>& OutShapes, int32* OutSkippedShapes)
    {
        using namespace UBot::Physics::Private;

        OutShapes.Reset();
        if (OutSkippedShapes != nullptr)
        {
            *OutSkippedShapes = 0;
        }

        if (Component.IsA<USkinnedMeshComponent>() || Component.IsA<UInstancedStaticMeshComponent>())
        {
            return false;
        }

        // GetBodySetup is not const: the shape components (box, sphere, capsule) bring their setup up to date there.
        const UBodySetup* BodySetup = const_cast<UPrimitiveComponent&>(Component).GetBodySetup();
        if (BodySetup == nullptr)
        {
            return false;
        }

        // The scale of the body as the engine builds the Chaos geometry: a scale that is almost zero becomes 0.1.
        FVector Scale3D = Component.GetComponentTransform().GetScale3D();
        if (Scale3D.IsNearlyZero())
        {
            Scale3D = FVector(0.1);
        }

        // Convex hulls and triangle meshes are scaled per axis by a scale that keeps its sign and is never zero.
        FVector NetScale = Scale3D;
        NetScale.X = FMath::Abs(NetScale.X) < UE_KINDA_SMALL_NUMBER ? UE_KINDA_SMALL_NUMBER : NetScale.X;
        NetScale.Y = FMath::Abs(NetScale.Y) < UE_KINDA_SMALL_NUMBER ? UE_KINDA_SMALL_NUMBER : NetScale.Y;
        NetScale.Z = FMath::Abs(NetScale.Z) < UE_KINDA_SMALL_NUMBER ? UE_KINDA_SMALL_NUMBER : NetScale.Z;

        // Chaos builds the simple shapes unless the setup uses complex collision as simple and has its triangle
        // meshes; in that case the triangle meshes are what it simulates.
        const bool bUseTriangleMeshes = BodySetup->GetCollisionTraceFlag() == CTF_UseComplexAsSimple && !BodySetup->TriMeshGeometries.IsEmpty();
        const FKAggregateGeom& Geometry = BodySetup->AggGeom;
        int32 Skipped = 0;

        if (bUseTriangleMeshes)
        {
            for (const Chaos::FTriangleMeshImplicitObjectPtr& TriangleMesh : BodySetup->TriMeshGeometries)
            {
                const Chaos::FTriangleMeshImplicitObject* Mesh = TriangleMesh.GetReference();
                TArray<FVector> Vertices;
                TArray<int32> Indices;
                FUBotPhysicsShape Shape;
                if (Mesh != nullptr)
                {
                    ReadChaosTriangleMesh(*Mesh, Vertices, Indices);
                }
                if (Mesh != nullptr && MakeTriangleMeshShape(Vertices, Indices, NetScale, Shape))
                {
                    OutShapes.Add(MoveTemp(Shape));
                }
                else
                {
                    ++Skipped;
                }
            }
        }
        else
        {
            for (const FKSphereElem& Element : Geometry.SphereElems)
            {
                const FKSphereElem Scaled = Element.GetFinalScaled(Scale3D, FTransform::Identity);
                FUBotPhysicsShape& Shape = OutShapes.AddDefaulted_GetRef();
                Shape.Type = EUBotPhysicsShapeType::Sphere;
                Shape.LocalTransform = FTransform(UBot::Units::CentimetersToMeters(Scaled.Center));
                Shape.Size = FVector(UBot::Units::CentimetersToMeters(FMath::Max(Scaled.Radius, UE_KINDA_SMALL_NUMBER)), 0.0, 0.0);
            }

            for (const FKBoxElem& Element : Geometry.BoxElems)
            {
                // X, Y and Z of a box element are full extents.
                const FKBoxElem Scaled = Element.GetFinalScaled(Scale3D, FTransform::Identity);
                const FVector HalfExtentsCentimeters(
                    FMath::Max(Scaled.X * 0.5f, UE_KINDA_SMALL_NUMBER),
                    FMath::Max(Scaled.Y * 0.5f, UE_KINDA_SMALL_NUMBER),
                    FMath::Max(Scaled.Z * 0.5f, UE_KINDA_SMALL_NUMBER));
                FUBotPhysicsShape& Shape = OutShapes.AddDefaulted_GetRef();
                Shape.Type = EUBotPhysicsShapeType::Box;
                Shape.LocalTransform = FTransform(Scaled.Rotation.Quaternion(), UBot::Units::CentimetersToMeters(Scaled.Center));
                Shape.Size = UBot::Units::CentimetersToMeters(HalfExtentsCentimeters);
            }

            for (const FKSphylElem& Element : Geometry.SphylElems)
            {
                // Length is the distance between the two sphere centers; the axis is the element's Z.
                const FKSphylElem Scaled = Element.GetFinalScaled(Scale3D, FTransform::Identity);
                FUBotPhysicsShape& Shape = OutShapes.AddDefaulted_GetRef();
                Shape.Type = EUBotPhysicsShapeType::Capsule;
                Shape.LocalTransform = FTransform(Scaled.Rotation.Quaternion(), UBot::Units::CentimetersToMeters(Scaled.Center));
                Shape.Size = FVector(
                    UBot::Units::CentimetersToMeters(FMath::Max(Scaled.Radius, UE_KINDA_SMALL_NUMBER)),
                    0.0,
                    UBot::Units::CentimetersToMeters(FMath::Max(Scaled.Length * 0.5f, UE_KINDA_SMALL_NUMBER)));
            }

            for (const FKConvexElem& Element : Geometry.ConvexElems)
            {
                if (Element.VertexData.Num() < 4)
                {
                    ++Skipped;
                    continue;
                }

                // The vertices of a hull are in the frame of the element; the cooked Chaos hull has the element's
                // transform applied, and the scale of the body comes on top.
                FTransform ElementTransform = Element.GetTransform();
                if (!ElementTransform.IsValid())
                {
                    ElementTransform = FTransform::Identity;
                }
                FUBotPhysicsShape& Shape = OutShapes.AddDefaulted_GetRef();
                Shape.Type = EUBotPhysicsShapeType::Convex;
                Shape.Vertices.Reserve(Element.VertexData.Num());
                for (const FVector& Vertex : Element.VertexData)
                {
                    Shape.Vertices.Add(UBot::Units::CentimetersToMeters(NetScale * ElementTransform.TransformPosition(Vertex)));
                }
            }

            // Chaos does not simulate tapered capsules and level sets as such: they are left out.
            Skipped += Geometry.TaperedCapsuleElems.Num() + Geometry.LevelSetElems.Num() + Geometry.SkinnedLevelSetElems.Num();
        }

        if (OutSkippedShapes != nullptr)
        {
            *OutSkippedShapes = Skipped;
        }
        return !OutShapes.IsEmpty();
    }
}
