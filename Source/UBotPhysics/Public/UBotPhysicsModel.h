#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "UBotPhysicsModel.generated.h"

/** Common collision and display geometry. Values are persistent; append only. */
UENUM(BlueprintType)
enum class EUBotPhysicsGeometryType : uint8
{
    Sphere, Box, Capsule, Convex, TriangleMesh, Cylinder, Ellipsoid, Plane, HeightField
};

/** Generalized-coordinate joints. A body's parent is declared by the body, not by the joint. */
UENUM(BlueprintType)
enum class EUBotPhysicsJointType : uint8
{
    Fixed, Hinge, Slide, Ball, Free
};

/** Physical drive model; the controller which supplies its input lives outside Physics. */
UENUM(BlueprintType)
enum class EUBotPhysicsDriveType : uint8
{
    Effort, Position, Velocity
};

/** Backend-owned exclusive model data. Common fields must not also be stored as editable overrides here. */
UCLASS(Abstract, BlueprintType, EditInlineNew, DefaultToInstanced)
class UBOTPHYSICS_API UUBotPhysicsModelExtension : public UObject
{
    GENERATED_BODY()
public:
    /** Required backend. None is invalid for an extension. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FName Backend;
};

/** A rigid body, relative to its parent (empty parent means the model frame), SI and UE axes. */
USTRUCT(BlueprintType)
struct UBOTPHYSICS_API FUBotPhysicsModelBody
{
    GENERATED_BODY()
    /** Stable, case-sensitive element identity within the model. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString Id;
    /** Stable parent body identity; empty means the model frame. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString ParentId;
    /** Authored local pose, meters, UE axes, unit scale. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FTransform LocalTransform = FTransform::Identity;
    /** Mass in kilograms. Zero is permitted for static bodies only. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    double MassKilograms = 1.0;
    /** Inertial frame relative to the body, translation in meters. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FTransform InertialTransform = FTransform::Identity;
    /** Principal moments about the inertial frame, kg m squared. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FVector InertiaKilogramMetersSquared = FVector(0.01);
    /** This body is driven by kinematic targets instead of integration. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    bool bKinematic = false;
};

/** Shape geometry shared by compilation and presentation; native material semantics belong to the extension. */
USTRUCT(BlueprintType)
struct UBOTPHYSICS_API FUBotPhysicsModelGeometry
{
    GENERATED_BODY()
    /** Stable geometry identity within the model. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString Id;
    /** Body identity; empty means fixed to the model frame. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString BodyId;
    /** Geometry kind. Mesh collisions may require an explicit convex approximation policy. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    EUBotPhysicsGeometryType Type = EUBotPhysicsGeometryType::Sphere;
    /** Shape pose relative to its body, meters, unit scale. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FTransform LocalTransform = FTransform::Identity;
    /** Sphere radius X; box/ellipsoid half extents; capsule/cylinder radius X and half length Z, meters. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FVector Size = FVector(0.1);
    /** Mesh or heightfield asset name within the model's backend resource package. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString Resource;
    /** Explicit vertices, in the shape frame, meters. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<FVector> Vertices;
    /** Triangle vertex indices. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<int32> Indices;
    /** Whether this shape participates in collision. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    bool bCollision = true;
    /** Display color; this does not define physical contact parameters. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FLinearColor Color = FLinearColor::Gray;
};

/** A joint attached to a body; several scalar joints on one body are permitted. */
USTRUCT(BlueprintType)
struct UBOTPHYSICS_API FUBotPhysicsModelJoint
{
    GENERATED_BODY()
    /** Stable joint identity. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString Id;
    /** Body this joint moves relative to its parent. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString BodyId;
    /** Generalized-coordinate joint type. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    EUBotPhysicsJointType Type = EUBotPhysicsJointType::Hinge;
    /** Joint origin in the body's frame, meters. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FVector PositionMeters = FVector::ZeroVector;
    /** Unit joint axis in the body frame; axial for hinge, polar for slide. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FVector Axis = FVector::UpVector;
    /** Lower and upper limits, radians or meters. Ball: zero lower and angular cone upper; ignored unless Limited. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FVector2D Limits = FVector2D::ZeroVector;
    /** Apply the scalar limits. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    bool bLimited = false;
    /** Initial absolute authored coordinates (no implicit reference subtraction). Scalar=1, ball=4 (xyzw), free=7 (xyz,xyzw), fixed=0. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<double> InitialPosition;
    /** Initial generalized velocities. Scalar=1, ball=3, free=6 (linear,angular), fixed=0. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<double> InitialVelocity;
};

/** An explicitly authored scalar drive. Native actuators must not be generated a second time here. */
USTRUCT(BlueprintType)
struct UBOTPHYSICS_API FUBotPhysicsModelDrive
{
    GENERATED_BODY()
    /** Stable input/drive identity. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString Id;
    /** Driven hinge or slide joint identity. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FString JointId;
    /** Physical drive response. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    EUBotPhysicsDriveType Type = EUBotPhysicsDriveType::Effort;
    /** Position gain in N/m or Nm/rad. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    double Stiffness = 0.0;
    /** Velocity gain in Ns/m or Nms/rad. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    double Damping = 0.0;
};

/** Immutable while instantiated. This and its exclusive extension are the only runtime compilation input. */
USTRUCT(BlueprintType)
struct UBOTPHYSICS_API FUBotPhysicsModel
{
    GENERATED_BODY()
    /** Asset revision; instances retain this revision until removed. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FGuid Revision;
    /** Rigid bodies in the model frame. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<FUBotPhysicsModelBody> Bodies;
    /** Collision and display geometries. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<FUBotPhysicsModelGeometry> Geometries;
    /** Joint definitions and initial coordinates. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<FUBotPhysicsModelJoint> Joints;
    /** Common scalar physical drives. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    TArray<FUBotPhysicsModelDrive> Drives;
    /** Exclusive native semantics, owned by the backend plugin. */
    UPROPERTY(VisibleAnywhere, Instanced, BlueprintReadOnly, Category = "Physics")
    TObjectPtr<UUBotPhysicsModelExtension> Extension;

    /** Structural and numeric checks. No World or backend is accessed. */
    bool Validate(FText& OutError) const;
};

/** Imported model asset. Change imported physics through reimport; live models retain their revision. */
UCLASS(BlueprintType)
class UBOTPHYSICS_API UUBotPhysicsModelAsset : public UDataAsset
{
    GENERATED_BODY()
public:
    /** The sole runtime compilation definition. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Physics")
    FUBotPhysicsModel Model;
};

/** Small value builder for programmatically authored common models. Finalize validates before publishing. */
class UBOTPHYSICS_API FUBotPhysicsModelBuilder
{
public:
    FUBotPhysicsModel Model;
    bool Finalize(FUBotPhysicsModel& OutModel, FText& OutError) const;
};

/** Read-only state of one joint at an integration endpoint, with independent position/velocity layouts. */
struct FUBotPhysicsJointState
{
    TArray<double> Position;
    TArray<double> Velocity;
    double TimestampSeconds = 0.0;
};

/** Named physical input. Native multi-input actuators retain their complete input vector. */
struct FUBotPhysicsActuatorInput
{
    FString Id;
    TArray<double> Values;
};

/** One externally applied body wrench, world frame, at the body origin (N, Nm). */
struct FUBotPhysicsBodyWrench
{
    FString BodyId;
    FVector ForceNewtons = FVector::ZeroVector;
    FVector TorqueNewtonMeters = FVector::ZeroVector;
};

/** Generalized effort at one joint, in velocity-coordinate order: N for linear and Nm for angular DOFs. */
struct FUBotPhysicsJointForce
{
    FString JointId;
    TArray<double> Values;
};

/** Externally prescribed body pose, world frame, SI units and UE axes. */
struct FUBotPhysicsKinematicTarget
{
    FString BodyId;
    FTransform WorldTransform = FTransform::Identity;
};

/** Explicit physical inputs; no controller, command timeout or robot behavior is represented here. */
struct FUBotPhysicsInputs
{
    TArray<FUBotPhysicsActuatorInput> Actuators;
    TArray<FUBotPhysicsBodyWrench> BodyWrenches;
    TArray<FUBotPhysicsJointForce> GeneralizedForces;
    TArray<FUBotPhysicsKinematicTarget> KinematicTargets;
};
