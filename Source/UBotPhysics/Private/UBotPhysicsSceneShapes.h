#pragma once

#include "CoreMinimal.h"
#include "UBotPhysicsScene.h"

namespace UBot::Physics::Private
{
    /**
     * Triangle mesh shape from triangle data in the frame of a body setup (centimeters): every vertex is scaled
     * per axis by Scale3D (the sign is kept) and converted to meters. A scale that mirrors (an odd number of
     * negative components) turns the triangles inside out, so the winding of every triangle is swapped to keep
     * the outside facing out. Triangles with an index outside Vertices are left out. LocalTransform is identity.
     * Returns false when no triangle is left.
     */
    bool MakeTriangleMeshShape(TConstArrayView<FVector> VerticesCentimeters, TConstArrayView<int32> Indices, const FVector& Scale3D, FUBotPhysicsShape& OutShape);
}
