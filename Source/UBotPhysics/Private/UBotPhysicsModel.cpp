#include "UBotPhysicsModel.h"

namespace
{
    bool ValidPose(const FTransform& Pose)
    {
        return !Pose.ContainsNaN() && Pose.GetRotation().IsNormalized()
            && Pose.GetScale3D().Equals(FVector::OneVector, 1.e-6);
    }

    bool FailModel(FText& Error, const FString& Reason)
    {
        Error = FText::FromString(Reason);
        return false;
    }
}

bool FUBotPhysicsModel::Validate(FText& OutError) const
{
    OutError = FText::GetEmpty();
    if (Bodies.IsEmpty())
    {
        return FailModel(OutError, TEXT("A physics model needs at least one body."));
    }
    UBot::Physics::TModelIdMap<const FUBotPhysicsModelBody*> BodyMap;
    for (const FUBotPhysicsModelBody& Body : Bodies)
    {
        if (Body.Id.IsEmpty() || BodyMap.Contains(Body.Id))
        {
            return FailModel(OutError, TEXT("Body identities must be nonempty and unique."));
        }
        if (!ValidPose(Body.LocalTransform) || !ValidPose(Body.InertialTransform)
            || !FMath::IsFinite(Body.MassKilograms) || Body.MassKilograms < 0.0
            || Body.InertiaKilogramMetersSquared.ContainsNaN() || Body.InertiaKilogramMetersSquared.GetMin() < 0.0
            || (Body.MassKilograms > 0.0 && Body.InertiaKilogramMetersSquared.GetMin() <= 0.0))
        {
            return FailModel(OutError, FString::Printf(TEXT("Body '%s' has invalid SI pose, mass or inertia."), *Body.Id));
        }
        const FVector& I = Body.InertiaKilogramMetersSquared;
        if (Body.MassKilograms > 0.0 && (I.X > I.Y + I.Z + 1.e-12 || I.Y > I.X + I.Z + 1.e-12 || I.Z > I.X + I.Y + 1.e-12))
        {
            return FailModel(OutError, FString::Printf(TEXT("Body '%s' violates the inertia triangle inequality."), *Body.Id));
        }
        BodyMap.Add(Body.Id, &Body);
    }
    for (const FUBotPhysicsModelBody& Body : Bodies)
    {
        UBot::Physics::FModelIdSet Seen;
        const FUBotPhysicsModelBody* Current = &Body;
        while (Current != nullptr && !Current->ParentId.IsEmpty())
        {
            if (Seen.Contains(Current->Id))
            {
                return FailModel(OutError, TEXT("Body parent graph contains a cycle."));
            }
            Seen.Add(Current->Id);
            const FUBotPhysicsModelBody* const* Parent = BodyMap.Find(Current->ParentId);
            if (Parent == nullptr)
            {
                return FailModel(OutError, FString::Printf(TEXT("Body '%s' refers to an unknown parent."), *Current->Id));
            }
            Current = *Parent;
        }
    }

    UBot::Physics::FModelIdSet GeometryIds;
    for (const FUBotPhysicsModelGeometry& Geometry : Geometries)
    {
        if (Geometry.Id.IsEmpty() || GeometryIds.Contains(Geometry.Id)
            || (!Geometry.BodyId.IsEmpty() && !BodyMap.Contains(Geometry.BodyId))
            || !ValidPose(Geometry.LocalTransform) || Geometry.Size.ContainsNaN())
        {
            return FailModel(OutError, TEXT("Geometry has an invalid identity, body, pose or size."));
        }
        GeometryIds.Add(Geometry.Id);
        const bool bMesh = Geometry.Type == EUBotPhysicsGeometryType::Convex || Geometry.Type == EUBotPhysicsGeometryType::TriangleMesh;
        if (bMesh)
        {
            if (Geometry.Vertices.IsEmpty() && Geometry.Resource.IsEmpty())
            {
                return FailModel(OutError, TEXT("Mesh geometry requires vertices or a resource."));
            }
            for (const FVector& Vertex : Geometry.Vertices)
            {
                if (Vertex.ContainsNaN())
                {
                    return FailModel(OutError, TEXT("Mesh geometry contains a non-finite vertex."));
                }
            }
            if (Geometry.Indices.Num() % 3 != 0)
            {
                return FailModel(OutError, TEXT("Triangle indices must contain complete triangles."));
            }
            for (const int32 Index : Geometry.Indices)
            {
                if (!Geometry.Vertices.IsValidIndex(Index))
                {
                    return FailModel(OutError, TEXT("Mesh geometry has an invalid triangle index."));
                }
            }
        }
        else if (Geometry.Type == EUBotPhysicsGeometryType::HeightField)
        {
            if (Geometry.Resource.IsEmpty())
            {
                return FailModel(OutError, TEXT("Heightfield geometry requires a resource."));
            }
        }
        else if (Geometry.Type != EUBotPhysicsGeometryType::Plane)
        {
            const bool bThreeAxes = Geometry.Type == EUBotPhysicsGeometryType::Box || Geometry.Type == EUBotPhysicsGeometryType::Ellipsoid;
            const bool bLength = Geometry.Type == EUBotPhysicsGeometryType::Capsule || Geometry.Type == EUBotPhysicsGeometryType::Cylinder;
            if (Geometry.Size.X <= 0.0 || (bThreeAxes && Geometry.Size.GetMin() <= 0.0) || (bLength && Geometry.Size.Z < 0.0))
            {
                return FailModel(OutError, TEXT("Primitive geometry has invalid dimensions."));
            }
        }
    }

    UBot::Physics::TModelIdMap<const FUBotPhysicsModelJoint*> JointMap;
    UBot::Physics::FModelIdSet FreeBodies;
    for (const FUBotPhysicsModelJoint& Joint : Joints)
    {
        if (Joint.Id.IsEmpty() || JointMap.Contains(Joint.Id) || !BodyMap.Contains(Joint.BodyId)
            || Joint.PositionMeters.ContainsNaN() || Joint.Axis.ContainsNaN())
        {
            return FailModel(OutError, TEXT("Joint has an invalid identity, body, origin or axis."));
        }
        const bool bScalar = Joint.Type == EUBotPhysicsJointType::Hinge || Joint.Type == EUBotPhysicsJointType::Slide;
        const bool bBall = Joint.Type == EUBotPhysicsJointType::Ball;
        if ((bScalar && !Joint.Axis.IsNormalized()) || (Joint.bLimited && ((!bScalar && !bBall) || Joint.Limits.ContainsNaN() || Joint.Limits.X > Joint.Limits.Y || (bBall && Joint.Limits.X != 0.0))))
        {
            return FailModel(OutError, TEXT("Joint axis or scalar limits are invalid."));
        }
        const int32 PositionCount = bScalar ? 1 : Joint.Type == EUBotPhysicsJointType::Ball ? 4 : Joint.Type == EUBotPhysicsJointType::Free ? 7 : 0;
        const int32 VelocityCount = bScalar ? 1 : Joint.Type == EUBotPhysicsJointType::Ball ? 3 : Joint.Type == EUBotPhysicsJointType::Free ? 6 : 0;
        if ((!Joint.InitialPosition.IsEmpty() && Joint.InitialPosition.Num() != PositionCount)
            || (!Joint.InitialVelocity.IsEmpty() && Joint.InitialVelocity.Num() != VelocityCount))
        {
            return FailModel(OutError, TEXT("Joint initial coordinates do not match its position/velocity layout."));
        }
        for (const double Value : Joint.InitialPosition)
        {
            if (!FMath::IsFinite(Value)) { return FailModel(OutError, TEXT("Initial positions must be finite.")); }
        }
        for (const double Value : Joint.InitialVelocity)
        {
            if (!FMath::IsFinite(Value)) { return FailModel(OutError, TEXT("Initial velocities must be finite.")); }
        }
        if (!Joint.InitialPosition.IsEmpty() && (Joint.Type == EUBotPhysicsJointType::Ball || Joint.Type == EUBotPhysicsJointType::Free))
        {
            const int32 Offset = Joint.Type == EUBotPhysicsJointType::Free ? 3 : 0;
            const FQuat Q(Joint.InitialPosition[Offset], Joint.InitialPosition[Offset + 1], Joint.InitialPosition[Offset + 2], Joint.InitialPosition[Offset + 3]);
            if (!Q.IsNormalized()) { return FailModel(OutError, TEXT("Initial joint quaternion must be normalized.")); }
        }
        if (Joint.Type == EUBotPhysicsJointType::Free)
        {
            if (!BodyMap[Joint.BodyId]->ParentId.IsEmpty() || FreeBodies.Contains(Joint.BodyId))
            {
                return FailModel(OutError, TEXT("A free joint must uniquely move a root body."));
            }
            FreeBodies.Add(Joint.BodyId);
        }
        JointMap.Add(Joint.Id, &Joint);
    }
    for (const FUBotPhysicsModelJoint& Joint : Joints)
    {
        if (FreeBodies.Contains(Joint.BodyId) && Joint.Type != EUBotPhysicsJointType::Free)
        {
            return FailModel(OutError, TEXT("A free joint cannot share a body with other joints."));
        }

    }
    UBot::Physics::FModelIdSet DriveIds;
    for (const FUBotPhysicsModelDrive& Drive : Drives)
    {
        const FUBotPhysicsModelJoint* const* Joint = JointMap.Find(Drive.JointId);
        if (Drive.Id.IsEmpty() || DriveIds.Contains(Drive.Id) || Joint == nullptr
            || ((*Joint)->Type != EUBotPhysicsJointType::Hinge && (*Joint)->Type != EUBotPhysicsJointType::Slide)
            || !FMath::IsFinite(Drive.Stiffness) || Drive.Stiffness < 0.0 || !FMath::IsFinite(Drive.Damping) || Drive.Damping < 0.0)
        {
            return FailModel(OutError, TEXT("A common drive requires a unique identity, scalar joint and finite nonnegative gains."));
        }
        DriveIds.Add(Drive.Id);
    }
    if (Extension != nullptr && Extension->Backend.IsNone())
    {
        return FailModel(OutError, TEXT("A backend model extension must identify its required backend."));
    }
    return true;
}

bool FUBotPhysicsModelBuilder::Finalize(FUBotPhysicsModel& OutModel, FText& OutError) const
{
    if (!Model.Validate(OutError))
    {
        return false;
    }
    FUBotPhysicsModel Result = Model;
    Result.Revision = FGuid::NewGuid();
    OutModel = MoveTemp(Result);
    return true;
}
