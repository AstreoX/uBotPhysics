#pragma once

#include "CoreMinimal.h"
#include "UBotBodyState.h"
#include "UBotExtensionRegistry.h"
#include "UBotPhysicsScene.h"
#include "UBotPhysicsTypes.h"
#include "UBotPhysicsModel.h"
#include "UObject/WeakObjectPtrTemplates.h"

class UPrimitiveComponent;
class UUBotPhysicsBindingComponent;
class UWorld;

/** One body handed to a solver. */
struct FUBotPhysicsBodyDesc
{
    /** UE component that shows the body. Already kinematic (not simulated by Chaos) when the solver sees it. */
    TWeakObjectPtr<UPrimitiveComponent> Component;

    /** Body name in the backend's model; None lets the backend derive it. */
    FName BodyName;

    /** The binding that handed the body over; backends may cast it to their own subclass for model settings. */
    TWeakObjectPtr<UUBotPhysicsBindingComponent> Binding;

    /** Component world transform at hand-over, UE world (cm). */
    FTransform InitialWorldTransform;
};

/** A displayed body. The full model may also contain bodies without a UE display component. */
struct FUBotPhysicsModelBodyBindingDesc
{
    FString BodyId;
    FUBotPhysicsBodyDesc Desc;
};

/** Atomic registration of one immutable model in this World's backend. All transforms here are SI. */
struct FUBotPhysicsModelInstanceDesc
{
    int64 InstanceId = INDEX_NONE;
    FUBotPhysicsModel Model;
    FTransform WorldTransform = FTransform::Identity;
    TArray<FUBotPhysicsModelBodyBindingDesc> BodyBindings;
    /** Adapter lifetime guard for body-only default registration; must not mutate anything. Never retained. */
    TFunction<bool()> IsCurrent;
};

/** Handles of displayed bodies after a successful atomic registration. */
struct FUBotPhysicsModelBodyHandle
{
    FString BodyId;
    int32 Handle = INDEX_NONE;
};

struct FUBotPhysicsModelInstanceResult
{
    TArray<FUBotPhysicsModelBodyHandle> Bodies;
};

/** Budget overflow semantics. Suspend preserves continuous native time and requires explicit Reset. */
enum class EUBotPhysicsOverrunPolicy : uint8
{
    Drop,
    Suspend
};

/**
 * One backend's simulation of one world. Created by its factory, owned by UUBotPhysicsSubsystem, game thread
 * only. Experimental: the interface may change between minor versions while it has few implementations.
 *
 * The subsystem calls Step on the uBot sim clock (see UUBotPhysicsSubsystem) and writes the poses of the
 * bodies back to their components after the steps of a frame. Before the steps it can hand the solver the
 * rest of the UE scene as static and kinematic bodies (UpdateScene). ApplyInputs only accepts physical
 * inputs. Controller algorithms, trajectories and command-loss policies belong to external components.
 *
 * Re-entry is not supported: the callbacks of a solver and of its factory (CreateSolver, AddModel, RemoveModel, AddBody, RemoveBody,
 * Step, GetBodyState, UpdateScene) must not bind or unbind bodies, and must not register or unregister backends,
 * synchronously (directly or through code they call). The subsystem defends against the known cases (a
 * backend that unregisters or replaces itself inside CreateSolver or AddBody, a bind that comes in while
 * CreateSolver runs, bodies that are unbound or rebound by a transform callback during the write-back), but
 * that is a safety net, not a contract. Do such things after the callback returned, e.g. on a later tick.
 */
class UBOTPHYSICS_API IUBotPhysicsSolver
{
public:
    virtual ~IUBotPhysicsSolver() = default;

    /**
     * Registers a whole model, all or nothing. Failure leaves every pre-existing model unchanged.
     * The default supports body-only backends through AddBody. Implementations copy definitions before returning.
     */
    virtual bool AddModel(const FUBotPhysicsModelInstanceDesc& Desc, FUBotPhysicsModelInstanceResult& OutResult, FText& OutError);

    /** Removes the whole instance. Failure means this backend must be suspended before ownership is returned. */
    virtual bool RemoveModel(int64 InstanceId, FText& OutError);

    /** Takes over a body. Returns a handle >= 0, or INDEX_NONE with OutError set. */
    virtual int32 AddBody(const FUBotPhysicsBodyDesc& Desc, FText& OutError) { OutError = INVTEXT("This backend requires model registration."); return INDEX_NONE; }

    /** Releases a body; its handle becomes invalid. */
    virtual void RemoveBody(int32 BodyHandle) {}

    /**
     * Fixed step in seconds, constant for the solver's lifetime. Must lie within [0.0001, 100]
     * (the range of FUBotSampleScheduler rates); a solver outside it is refused when it is created.
     */
    virtual double GetStepSeconds() const = 0;

    /** Advances one step. A failed step must leave the last published state and timestamp available. */
    virtual bool Step(double StepEndTimeSeconds, FText& OutError) = 0;

    /** Continuous-time backends select Suspend; others retain the original bounded/drop schedule. */
    virtual EUBotPhysicsOverrunPolicy GetOverrunPolicy() const { return EUBotPhysicsOverrunPolicy::Drop; }

    /** Resets the complete backend scene and every instance. New native/uBot time epoch starts at SimTimeSeconds. */
    virtual bool Reset(double SimTimeSeconds, FText& OutError) { OutError = INVTEXT("This backend does not support reset."); return false; }

    /** Replaces held physical inputs for an instance. Invalid input is rejected as a whole. Game thread only. */
    virtual bool ApplyInputs(int64 InstanceId, const FUBotPhysicsInputs& Inputs, FText& OutError) { OutError = INVTEXT("This backend does not accept physical inputs."); return false; }

    /** Read-only joint endpoint state. False before the first step or when the instance/joint is unknown. */
    virtual bool GetJointState(int64 InstanceId, const FString& JointId, FUBotPhysicsJointState& OutState) const { return false; }

    /**
     * State at the end of the last step (FUBotBodyState, UE world axes, SI). False before any step or for an
     * unknown handle. The subsystem sets TimestampSeconds to the end time of that step, and Source to the
     * backend name when the solver leaves it None.
     */
    virtual bool GetBodyState(int32 BodyHandle, FUBotBodyState& OutState) const = 0;

    /**
     * The rest of the UE scene, as the backend's capabilities ask for it: bStaticSceneGeometry brings the
     * static colliders, bKinematicForeignBodies the moving ones that something else moves. Called at most
     * once per frame, before the frame's steps, when something changed or kinematic bodies exist; never for
     * a backend that declares neither capability. The default ignores it.
     *
     * Update.Removed is handled before Update.Added. An Id names one export of one component and nothing
     * else. A component that stops being part of the scene (destroyed, collision off, tagged out) or whose
     * body setup, world scale or mobility changes leaves under its Id and comes back under a new one. A body
     * that only dropped out of this solver's scene because this backend took the component over returns under
     * the Id it had. Shapes, units and frames are described at FUBotPhysicsShape;
     * UBot::Physics::GetCollisionShapes gives the same shapes for a backend's own bodies.
     */
    virtual bool UpdateScene(const FUBotPhysicsSceneUpdate& Update, FText& OutError) { return true; }

private:
    TMap<int64, TArray<int32>> BodyOnlyModels;
};

/**
 * Creates the solvers of one backend. Register one at the extension point UBot::Physics::BackendExtensionPoint;
 * GetExtensionName() is the backend name bindings use. Experimental.
 */
class UBOTPHYSICS_API IUBotPhysicsBackendFactory : public IUBotExtension
{
public:
    virtual FUBotPhysicsCapabilities GetCapabilities() const = 0;

    /**
     * A new solver for World, or nullptr when the backend cannot run there (the backend logs why). Must not bind
     * or unbind bodies or (un)register backends before it returns (see IUBotPhysicsSolver).
     */
    virtual TSharedPtr<IUBotPhysicsSolver> CreateSolver(UWorld& World) = 0;
};

namespace UBot::Physics
{
    /** The backend factory registered under BackendName at BackendExtensionPoint, or nullptr. */
    UBOTPHYSICS_API TSharedPtr<IUBotPhysicsBackendFactory> FindBackendFactory(FName BackendName);

    /** Registered backend names in registration order. */
    UBOTPHYSICS_API TArray<FName> GetBackendNames();
}
