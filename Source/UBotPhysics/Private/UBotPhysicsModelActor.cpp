#include "UBotPhysicsModelActor.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"
#include "UBotPhysics.h"
#include "UBotPhysicsBindingComponent.h"

namespace
{
    FTransform DisplayTransform(const FTransform& SI)
    {
        return FTransform(SI.GetRotation(), SI.GetLocation() * 100.0);
    }

    void AddQuad(TArray<int32>& Indices, int32 A, int32 B, int32 C, int32 D)
    {
        Indices.Append({ A, B, C, A, C, D });
    }

    void BuildPrimitive(const FUBotPhysicsModelGeometry& Geometry, TArray<FVector>& Vertices, TArray<int32>& Indices)
    {
        if (!Geometry.Vertices.IsEmpty())
        {
            Vertices = Geometry.Vertices;
            Indices = Geometry.Indices;
            return;
        }
        const FVector S = Geometry.Size;
        if (Geometry.Type == EUBotPhysicsGeometryType::Box)
        {
            Vertices = { FVector(-S.X,-S.Y,-S.Z), FVector(S.X,-S.Y,-S.Z), FVector(S.X,S.Y,-S.Z), FVector(-S.X,S.Y,-S.Z),
                FVector(-S.X,-S.Y,S.Z), FVector(S.X,-S.Y,S.Z), FVector(S.X,S.Y,S.Z), FVector(-S.X,S.Y,S.Z) };
            AddQuad(Indices, 0,3,2,1); AddQuad(Indices, 4,5,6,7); AddQuad(Indices, 0,1,5,4);
            AddQuad(Indices, 1,2,6,5); AddQuad(Indices, 2,3,7,6); AddQuad(Indices, 3,0,4,7);
            return;
        }
        if (Geometry.Type == EUBotPhysicsGeometryType::Plane)
        {
            // An infinite physical plane is displayed as a 10 m patch when no display extent was authored.
            const double X = S.X > 0.0 ? S.X : 5.0;
            const double Y = S.Y > 0.0 ? S.Y : 5.0;
            Vertices = { FVector(-X,-Y,0), FVector(X,-Y,0), FVector(X,Y,0), FVector(-X,Y,0) };
            AddQuad(Indices, 0,1,2,3);
            return;
        }
        constexpr int32 Segments = 32;
        if (Geometry.Type == EUBotPhysicsGeometryType::Cylinder)
        {
            for (int32 Ring = 0; Ring < 2; ++Ring)
            {
                for (int32 Segment = 0; Segment < Segments; ++Segment)
                {
                    const double Angle = 2.0 * PI * Segment / Segments;
                    Vertices.Add(FVector(S.X * FMath::Cos(Angle), S.X * FMath::Sin(Angle), Ring == 0 ? -S.Z : S.Z));
                }
            }
            for (int32 Segment = 0; Segment < Segments; ++Segment)
            {
                const int32 Next = (Segment + 1) % Segments;
                AddQuad(Indices, Segment, Next, Next + Segments, Segment + Segments);
                if (Segment > 0 && Segment < Segments - 1)
                {
                    Indices.Append({ 0, Segment + 1, Segment, Segments, Segment + Segments, Segment + Segments + 1 });
                }
            }
            return;
        }
        if (Geometry.Type != EUBotPhysicsGeometryType::Sphere && Geometry.Type != EUBotPhysicsGeometryType::Capsule
            && Geometry.Type != EUBotPhysicsGeometryType::Ellipsoid) { return; }
        constexpr int32 Rings = 16;
        const bool bCapsule = Geometry.Type == EUBotPhysicsGeometryType::Capsule;
        const int32 TotalRings = Rings + (bCapsule ? 1 : 0);
        for (int32 Ring = 0; Ring <= TotalRings; ++Ring)
        {
            const int32 SphereRing = bCapsule && Ring > Rings / 2 ? Ring - 1 : Ring;
            const double Latitude = PI * SphereRing / Rings;
            for (int32 Segment = 0; Segment < Segments; ++Segment)
            {
                const double Angle = 2.0 * PI * Segment / Segments;
                const double Sin = FMath::Sin(Latitude);
                const double Cos = FMath::Cos(Latitude);
                FVector Vertex(Sin * FMath::Cos(Angle), Sin * FMath::Sin(Angle), Cos);
                if (Geometry.Type == EUBotPhysicsGeometryType::Ellipsoid) { Vertex *= S; }
                else
                {
                    Vertex *= S.X;
                    if (Geometry.Type == EUBotPhysicsGeometryType::Capsule) { Vertex.Z += Ring <= Rings / 2 ? S.Z : -S.Z; }
                }
                Vertices.Add(Vertex);
            }
        }
        for (int32 Ring = 0; Ring < TotalRings; ++Ring)
        {
            for (int32 Segment = 0; Segment < Segments; ++Segment)
            {
                const int32 Next = (Segment + 1) % Segments;
                AddQuad(Indices, Ring * Segments + Segment, (Ring + 1) * Segments + Segment,
                    (Ring + 1) * Segments + Next, Ring * Segments + Next);
            }
        }
    }

    UStaticMesh* MakeDisplayMesh(UObject* Outer, const FUBotPhysicsModelGeometry& Geometry, UMaterialInterface* Material)
    {
        TArray<FVector> Vertices;
        TArray<int32> Indices;
        BuildPrimitive(Geometry, Vertices, Indices);
        if (Vertices.IsEmpty() || Indices.IsEmpty()) { return nullptr; }
        FMeshDescription Description;
        FStaticMeshAttributes Attributes(Description);
        Attributes.Register();
        auto Positions = Attributes.GetVertexPositions();
        auto Normals = Attributes.GetVertexInstanceNormals();
        auto Tangents = Attributes.GetVertexInstanceTangents();
        auto Signs = Attributes.GetVertexInstanceBinormalSigns();
        auto UVs = Attributes.GetVertexInstanceUVs();
        UVs.SetNumChannels(1);
        auto Colors = Attributes.GetVertexInstanceColors();
        TArray<FVertexID> VertexIds;
        for (const FVector& Point : Vertices)
        {
            const FVertexID Id = Description.CreateVertex();
            Positions[Id] = FVector3f(Point * 100.0);
            VertexIds.Add(Id);
        }
        const FPolygonGroupID Group = Description.CreatePolygonGroup();
        for (int32 Triangle = 0; Triangle + 2 < Indices.Num(); Triangle += 3)
        {
            const int32 A = Indices[Triangle], B = Indices[Triangle + 1], C = Indices[Triangle + 2];
            if (!VertexIds.IsValidIndex(A) || !VertexIds.IsValidIndex(B) || !VertexIds.IsValidIndex(C)) { return nullptr; }
            const FVector Normal = FVector::CrossProduct(Vertices[B] - Vertices[A], Vertices[C] - Vertices[A]).GetSafeNormal();
            if (Normal.IsNearlyZero()) { continue; }
            TArray<FVertexInstanceID> Corners;
            for (const int32 Index : { A, B, C })
            {
                const FVertexInstanceID Corner = Description.CreateVertexInstance(VertexIds[Index]);
                Normals[Corner] = FVector3f(Normal);
                Tangents[Corner] = FVector3f((Vertices[B] - Vertices[A]).GetSafeNormal());
                Signs[Corner] = 1.0f;
                UVs.Set(Corner, 0, FVector2f::ZeroVector);
                Colors[Corner] = FVector4f(Geometry.Color.R, Geometry.Color.G, Geometry.Color.B, Geometry.Color.A);
                Corners.Add(Corner);
            }
            Description.CreatePolygon(Group, Corners);
        }
        UStaticMesh* Mesh = NewObject<UStaticMesh>(Outer);
        Mesh->GetStaticMaterials().Add(FStaticMaterial(Material));
        UStaticMesh::FBuildMeshDescriptionsParams Parameters;
        Parameters.bFastBuild = true;
        Parameters.bBuildSimpleCollision = true;
        Parameters.bAllowCpuAccess = true;
        TArray<const FMeshDescription*> Descriptions = { &Description };
        Mesh->BuildFromMeshDescriptions(Descriptions, Parameters);
        return Mesh;
    }
}

AUBotPhysicsModelActor::AUBotPhysicsModelActor()
{
    PrimaryActorTick.bCanEverTick = false;
    SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("ModelRoot")));
    PhysicsBinding = CreateDefaultSubobject<UUBotPhysicsBindingComponent>(TEXT("PhysicsBinding"));
    static ConstructorHelpers::FObjectFinder<UMaterialInterface> ColorMaterial(TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial"));
    DisplayMaterial = ColorMaterial.Object;
}

void AUBotPhysicsModelActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (BuiltModelAsset != ModelAsset || (ModelAsset != nullptr && BuiltRevision != ModelAsset->Model.Revision)
        || BuiltBackend != PhysicsBackend || (ModelAsset != nullptr && GeneratedComponents.IsEmpty()))
    {
        RebuildModelDisplay();
    }
}

bool AUBotPhysicsModelActor::RebuildModelDisplay()
{
    if (PhysicsBinding->GetBindingStatus() == EUBotPhysicsBindingStatus::Bound) { return false; }
    for (UStaticMeshComponent* Component : GeneratedComponents)
    {
        if (IsValid(Component)) { Component->DestroyComponent(); }
    }
    GeneratedComponents.Reset();
    PhysicsBinding->Bodies.Reset();
    PhysicsBinding->Model = ModelAsset;
    BuiltModelAsset = ModelAsset;
    BuiltRevision = ModelAsset != nullptr ? ModelAsset->Model.Revision : FGuid();
    BuiltBackend = PhysicsBackend;
    if (ModelAsset == nullptr) { return true; }
    FText Error;
    if (!ModelAsset->Model.Validate(Error))
    {
        UE_LOG(LogUBotPhysics, Warning, TEXT("Cannot display physics model: %s"), *Error.ToString());
        return false;
    }
    PhysicsBinding->Backend = !PhysicsBackend.IsNone() ? PhysicsBackend
        : ModelAsset->Model.Extension != nullptr ? ModelAsset->Model.Extension->Backend : NAME_None;
    UBot::Physics::TModelIdMap<UStaticMeshComponent*> Bodies;
    for (const FUBotPhysicsModelBody& Body : ModelAsset->Model.Bodies)
    {
        UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(this, NAME_None, RF_Transactional);
        Component->CreationMethod = EComponentCreationMethod::Instance;
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        AddInstanceComponent(Component);
        Component->RegisterComponent();
        GeneratedComponents.Add(Component);
        Bodies.Add(Body.Id, Component);
        FUBotPhysicsBodyBinding& Mapping = PhysicsBinding->Bodies.AddDefaulted_GetRef();
        Mapping.ModelBodyId = Body.Id;
        Mapping.Component.OverrideComponent = Component;
    }
    for (const FUBotPhysicsModelBody& Body : ModelAsset->Model.Bodies)
    {
        UStaticMeshComponent* Component = Bodies[Body.Id];
        Component->AttachToComponent(Body.ParentId.IsEmpty() ? GetRootComponent() : Bodies[Body.ParentId], FAttachmentTransformRules::KeepRelativeTransform);
        Component->SetRelativeTransform(DisplayTransform(Body.LocalTransform));
    }
    for (const FUBotPhysicsModelGeometry& Geometry : ModelAsset->Model.Geometries)
    {
        UStaticMesh* Mesh = MakeDisplayMesh(this, Geometry, DisplayMaterial);
        if (Mesh == nullptr)
        {
            UE_LOG(LogUBotPhysics, Warning, TEXT("Physics model geometry '%s' has no display triangles; its physical definition is retained."), *Geometry.Id);
            continue;
        }
        UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(this, NAME_None, RF_Transactional);
        Component->CreationMethod = EComponentCreationMethod::Instance;
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetStaticMesh(Mesh);
        Component->ComponentTags.Add(UBot::Physics::ModelGeometryTag);
        Component->SetCollisionProfileName(UCollisionProfile::BlockAllDynamic_ProfileName);
        // Visual-only exterior geometry remains visible to scene queries without introducing physical
        // contact proxies or being exported as foreign physics scene geometry.
        if (!Geometry.bCollision) { Component->SetCollisionEnabled(ECollisionEnabled::QueryOnly); }
        Component->SetupAttachment(Geometry.BodyId.IsEmpty() ? GetRootComponent() : Bodies[Geometry.BodyId]);
        Component->SetRelativeTransform(DisplayTransform(Geometry.LocalTransform));
        AddInstanceComponent(Component);
        Component->RegisterComponent();
        GeneratedComponents.Add(Component);
    }
    return true;
}
