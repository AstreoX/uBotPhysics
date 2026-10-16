#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "UBotPhysicsTypes.generated.h"

/** What a physics backend can simulate. Fields are only ever appended. */
USTRUCT(BlueprintType)
struct UBOTPHYSICS_API FUBotPhysicsCapabilities
{
    GENERATED_BODY()

    /** Free rigid bodies. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bRigidBodies = false;

    /** Jointed bodies in reduced (generalized) coordinates. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bArticulations = false;

    /** Joints as constraints between free bodies. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bMaximalCoordinateJoints = false;

    /** Contacts between the backend's own bodies. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bContacts = false;

    /** Can collide with static UE geometry the subsystem hands over. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bStaticSceneGeometry = false;

    /** Can mirror bodies simulated elsewhere as kinematic bodies. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bKinematicForeignBodies = false;

    /** Body states carry linear and angular accelerations. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bAccelerations = false;

    /** The same inputs and steps give identical results. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bDeterministic = false;

    /** The simulation can be differentiated. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bDifferentiable = false;

    /** Deformable solid bodies. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bSoftBodies = false;

    /** Cloth. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bCloth = false;

    /** Particle systems. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    bool bParticles = false;
};

/** One UE component handed to a backend. */
USTRUCT(BlueprintType)
struct UBOTPHYSICS_API FUBotPhysicsBodyBinding
{
    GENERATED_BODY()

    /** Primitive component of the binding's own actor that shows the body; it must be Movable. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|Physics", meta = (UseComponentPicker, AllowedClasses = "/Script/Engine.PrimitiveComponent"))
    FComponentReference Component;

    /** Name of the body in the backend's model. None: the backend derives it. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "uBot|Physics")
    FName BodyName;
};

/** Where a physics binding stands. Append new values at the end. */
UENUM(BlueprintType)
enum class EUBotPhysicsBindingStatus : uint8
{
    /** Not bound: Bind was not called yet, or Unbind was. */
    Unbound,
    /** Every body is handed to the backend. */
    Bound,
    /** The last Bind failed: no backend with this name is registered. */
    BackendMissing,
    /** The last Bind failed: a component or the backend refused. Nothing is bound. */
    Rejected
};

/** What one backend currently sees of the UE scene. */
USTRUCT(BlueprintType)
struct UBOTPHYSICS_API FUBotPhysicsSceneStats
{
    GENERATED_BODY()

    /** Static colliders handed to the backend. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    int32 StaticBodies = 0;

    /** Kinematic (moving) colliders handed to the backend. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    int32 KinematicBodies = 0;

    /** Shapes of those bodies. */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    int32 Shapes = 0;

    /** Components of the world with physics collision that could not be exported (unsupported type or shapes, no body setup). */
    UPROPERTY(BlueprintReadOnly, Category = "uBot|Physics")
    int32 SkippedComponents = 0;
};
