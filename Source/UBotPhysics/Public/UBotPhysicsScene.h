#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"

class UPrimitiveComponent;

// What a physics backend sees of the rest of the UE scene: collision shapes of components and the
// static and kinematic bodies built from them. Plain C++, experimental: may change between minor versions.

/** Kinds of exported collision shapes. Append only. */
enum class EUBotPhysicsShapeType : uint8
{
    Sphere,
    Box,
    Capsule,
    Convex,
    TriangleMesh
};

/** One collision shape in the frame of its body: meters, UE axes, unit scale. */
struct FUBotPhysicsShape
{
    EUBotPhysicsShapeType Type = EUBotPhysicsShapeType::Sphere;

    /** Shape frame relative to the body frame. Capsule: its axis is the shape's Z. Convex and triangle mesh: identity. */
    FTransform LocalTransform = FTransform::Identity;

    /** Sphere: X = radius. Box: half extents. Capsule: X = radius, Z = half the distance between the two sphere centers. Meters; unused for meshes. */
    FVector Size = FVector::ZeroVector;

    /** Convex: points whose hull is the shape. Triangle mesh: vertices. Body frame, meters. */
    TArray<FVector> Vertices;

    /** Triangle mesh: three vertex indices per triangle. Empty for the other types. */
    TArray<int32> Indices;
};

namespace UBot::Physics
{
    /**
     * Collision shapes of Component as Chaos simulates it: the simple shapes of its body setup, and its
     * triangle meshes only when the body setup uses complex collision as simple. The component's world
     * scale is baked in the way UE does when it builds Chaos geometry (FKSphereElem/FKBoxElem/FKSphylElem::
     * GetFinalScaled; convex and mesh vertices scaled per axis; a mirroring scale swaps the winding of
     * mesh triangles). Shapes are in the component's frame without scale, meters. Collision settings are
     * not looked at. False when there is nothing to export: no body setup, only unsupported shapes, or an
     * unsupported component type (skinned meshes, instanced static meshes). OutSkippedShapes (optional)
     * counts shapes left out (tapered capsules, level sets).
     *
     * The shapes come in a fixed order: spheres, boxes, capsules, convex hulls, or the triangle meshes.
     * Experimental.
     */
    UBOTPHYSICS_API bool GetCollisionShapes(const UPrimitiveComponent& Component, TArray<FUBotPhysicsShape>& OutShapes, int32* OutSkippedShapes = nullptr);
}

/** How a scene body moves. */
enum class EUBotPhysicsSceneBodyKind : uint8
{
    /** Never moves: the component's mobility is Static or Stationary. */
    Static,
    /** Moves: the component's mobility is Movable and something else (Chaos, another backend, game code) moves it. */
    Kinematic
};

/** A UE component a backend sees but does not simulate. */
struct FUBotPhysicsSceneBody
{
    /** Names this export of the component for the subsystem's life; never names anything else. */
    int64 Id = INDEX_NONE;

    TWeakObjectPtr<UPrimitiveComponent> Component;

    EUBotPhysicsSceneBodyKind Kind = EUBotPhysicsSceneBodyKind::Static;

    /** Body frame in the world: the component's world transform without scale, meters. */
    FTransform WorldTransform = FTransform::Identity;

    TArray<FUBotPhysicsShape> Shapes;

    /** Of the component's simple physical material (the engine default when it has none). */
    float Friction = 0.0f;
    float Restitution = 0.0f;
};

/** Pose of a kinematic scene body. */
struct FUBotPhysicsScenePose
{
    int64 Id = INDEX_NONE;

    /** Meters, without scale. */
    FTransform WorldTransform = FTransform::Identity;
};

/** What changed in the scene one solver sees since its previous update. */
struct FUBotPhysicsSceneUpdate
{
    /** Sim time of the frame. The poses are the world's at the start of this frame's stepping. */
    double SimTimeSeconds = 0.0;

    /** Bodies that left this solver's scene. Handled before Added. */
    TArray<int64> Removed;

    TArray<FUBotPhysicsSceneBody> Added;

    /** Every kinematic body of this solver's scene after the update (Added ones included) with its pose. */
    TArray<FUBotPhysicsScenePose> KinematicPoses;
};
