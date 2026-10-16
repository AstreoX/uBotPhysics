#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineBaseTypes.h"
#include "Subsystems/WorldSubsystem.h"
#include "UBotBodyState.h"
#include "UBotPhysicsBackend.h"
#include "UBotSampleScheduler.h"
#include "UObject/ObjectKey.h"
#include "UBotPhysicsSubsystem.generated.h"

class UBodySetup;
class UPrimitiveComponent;
class UUBotPhysicsBindingComponent;
class UUBotPhysicsSubsystem;

/** Calls UUBotPhysicsSubsystem::AdvanceTo once per frame. Ticks in TG_PrePhysics and never while the game is paused. */
struct UBOTPHYSICS_API FUBotPhysicsTickFunction : public FTickFunction
{
    FUBotPhysicsTickFunction();

    /** The subsystem this function steps. */
    UUBotPhysicsSubsystem* Target = nullptr;

    /** How many times the engine ran this function; for diagnostics and tests. */
    int64 ExecutedCount = 0;

    //~ Begin FTickFunction interface
    virtual void ExecuteTick(float DeltaTime, ELevelTick TickType, ENamedThreads::Type CurrentThread, const FGraphEventRef& MyCompletionGraphEvent) override;
    virtual FString DiagnosticMessage() override;
    virtual FName DiagnosticContext(bool bDetailed) override;
    //~ End FTickFunction interface
};

/**
 * Runs the physics backends of one game or PIE world side by side with Chaos. It owns one solver per
 * backend, the table of which backend simulates which component, and the per-frame order: every frame in
 * TG_PrePhysics each solver is stepped on its own fixed step up to (never past) the uBot sim time, then the
 * poses of all bound bodies are written back to their components, attach parents before their children.
 * Components that are not bound stay with Chaos. A backend that asks for it also sees them, as static and
 * kinematic colliders (IUBotPhysicsSolver::UpdateScene).
 *
 * Bodies reach a backend through UUBotPhysicsBindingComponent; this subsystem is the C++ API behind it.
 * A backend that is unregistered while its bodies are bound loses them: they go back to Chaos, their
 * bindings become BackendMissing and its solver is destroyed.
 * Experimental: may change between minor versions.
 */
UCLASS()
class UBOTPHYSICS_API UUBotPhysicsSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    /** Physics subsystem of WorldContext's world; nullptr when there is no world or it is not a game/PIE world. */
    static UUBotPhysicsSubsystem* Get(const UObject* WorldContext);

    //~ Begin USubsystem interface
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;
    //~ End USubsystem interface

    //~ Begin UWorldSubsystem interface
    virtual void OnWorldBeginPlay(UWorld& InWorld) override;
    //~ End UWorldSubsystem interface

    /**
     * Hands the bodies of Binding to its backend, all or nothing, and tells Binding what became of it
     * (status, error, bound components). Returns true when bound (also when Binding already was). On
     * failure OutError says why and everything is rolled back. The order of checks is: the backend factory
     * is registered, each component resolves to a Movable primitive component of the binding's own actor
     * that is neither bound nor moved by another body state source, the backend creates its solver, the
     * backend accepts each body. The backend's code runs while the bodies are handed over and may
     * unregister or replace the backend; after the solver is created and after each body is accepted the
     * registered factory and the solver in the slot are checked, and if either changed the bodies accepted
     * so far go back and the binding fails with BackendMissing. While a backend's CreateSolver runs, another
     * BindBodies for the same backend fails with Rejected (its solver is still being created). The
     * backend's own callbacks must not bind, unbind or (un)register backends at all, see IUBotPhysicsSolver.
     */
    bool BindBodies(UUBotPhysicsBindingComponent& Binding, FText& OutError);

    /**
     * Takes Binding's bodies back from the backend. A component that was simulating by Chaos before the
     * hand-over simulates again, with the last linear and angular velocity of the backend. Resets Binding
     * to Unbound; does nothing else if it was not bound.
     */
    void UnbindBodies(UUBotPhysicsBindingComponent& Binding);

    /** Backend simulating Component, or NAME_None when Chaos (UE) does. */
    FName FindBodyBackend(const UPrimitiveComponent* Component) const;

    /**
     * State of a bound component after its backend's last step, UE world axes and SI. False if Component
     * is not bound or the backend has not stepped yet. TimestampSeconds is the sim time the last step
     * ended at; Source is the backend name unless the solver set one.
     */
    bool GetBodyState(const UPrimitiveComponent* Component, FUBotBodyState& OutState) const;

    /**
     * Steps every solver up to SimTimeSeconds, then writes the poses of all bound bodies back to their
     * components. Called once per frame by the tick function; public for tests.
     *
     * A solver's clock starts at the first call that finds it holding a body: its first step ends one
     * step later and every following step on that grid, so solver time never passes sim time and a pause
     * leaves no steps to catch up. At most GetMaxStepsPerFrame() steps run per call; the steps beyond
     * that are skipped and counted by GetDroppedSteps.
     *
     * The scene comes first: when some solver holds bodies and declares bStaticSceneGeometry or
     * bKinematicForeignBodies, the world's components are scanned once (the cost grows with their number) and
     * then, per solver that holds bodies, after its clock check and before its steps, also in a call where it
     * takes no step, the solver is sent what changed in its scene (IUBotPhysicsSolver::UpdateScene). Poses
     * are the components' at that moment, which is what the world shows when stepping starts: Chaos bodies
     * show the result of its previous physics frame. A component is part of the scene when it is registered,
     * has physics collision, carries no UBot::Physics::NoSceneTag (on itself or its actor) and
     * GetCollisionShapes can export it; a Static or Stationary one is a static body, a Movable one a
     * kinematic body. A solver does not see the components bound to its own backend or attached to them.
     *
     * The write-back happens once per call, after every solver stepped, for each body whose state is
     * available, also when its solver did not step in this call. Bodies are written in ascending attach
     * depth (a component before the components attached to it), across backends, so that a bound child
     * ends at its own solver pose even when its bound parent moved. Moving a component runs delegates
     * that may unbind, rebind or destroy bodies; a body is only written while the binding, backend, handle,
     * hand-over generation and solver it was read from still own it, checked before the pose and again
     * before the velocity. The generation is what tells a rebound body from the old one when the solver
     * hands out the released handle again.
     */
    void AdvanceTo(double SimTimeSeconds);

    /** The solver of Backend in this world, or nullptr if no body was bound to it yet or its backend was unregistered. Valid until the world ends or the backend is unregistered. */
    IUBotPhysicsSolver* FindSolver(FName Backend) const;

    /** Steps skipped because a frame needed more than GetMaxStepsPerFrame(); counted over the world's life. */
    int64 GetDroppedSteps(FName Backend) const;

    /**
     * What Backend's solver sees of the scene as of its last update: the static and kinematic bodies it was
     * sent and their shapes. SkippedComponents is the count of the last scan of the world: components with
     * physics collision that could not be exported. False (and all zero) when Backend has no solver in this world.
     */
    bool GetSceneStats(FName Backend, FUBotPhysicsSceneStats& OutStats) const;

    /** Most steps one solver runs in one frame. Default 1000. */
    int32 GetMaxStepsPerFrame() const { return MaxStepsPerFrame; }

    /** Clamped to at least 1. */
    void SetMaxStepsPerFrame(int32 Value);

    /** The per-frame tick function (for diagnostics and tests). It is registered when play begins. */
    const FUBotPhysicsTickFunction& GetTickFunction() const { return TickFunction; }

protected:
    //~ Begin UWorldSubsystem interface
    virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
    //~ End UWorldSubsystem interface

private:
    /** One backend's solver in this world and the clock that steps it. */
    struct FSolverSlot
    {
        FName Backend;
        TSharedPtr<IUBotPhysicsSolver> Solver;
        /** The factory that made Solver. A registered factory that is another object means the backend was replaced. */
        TWeakPtr<IUBotPhysicsBackendFactory> Factory;
        FUBotSampleScheduler Scheduler;
        /** Bodies the solver holds. */
        int32 BodyCount = 0;
        /** False until AdvanceTo starts the scheduler, and again when the solver loses its last body. */
        bool bClockRunning = false;
        /**
         * True while the factory's CreateSolver runs for this slot. A bind for the same backend that comes in
         * meanwhile (the factory's own code triggered it) is rejected: the solver does not exist yet.
         */
        bool bCreatingSolver = false;
        /** Steps the scheduler dropped in earlier runs of the clock. */
        int64 DroppedBefore = 0;
        bool bHasStepped = false;
        double LastStepEndSeconds = 0.0;
        bool bWarnedNonFinite = false;
        /** Read from the factory when the solver was made; a factory's capabilities do not change. Nothing while the slot has no solver. */
        FUBotPhysicsCapabilities Capabilities;
        /** Ids of the scene bodies the solver was told about and has not been told to drop. Cleared when the slot loses its solver. */
        TSet<int64> KnownSceneBodies;
        /** What the solver sees of the scene as of its last update (SkippedComponents is not kept here). */
        FUBotPhysicsSceneStats SceneStats;
    };

    /** A component handed to a backend. */
    struct FBodySlot
    {
        TWeakObjectPtr<UPrimitiveComponent> Component;
        FName Backend;
        int32 Handle = INDEX_NONE;
        /** The binding that handed the component over. */
        TWeakObjectPtr<UUBotPhysicsBindingComponent> Binding;
        /** Identifies the binding even after it is gone: a new object at the same address is another key. */
        TObjectKey<UUBotPhysicsBindingComponent> BindingKey;
        /**
         * Number of this hand-over, taken from NextBodyGeneration: every hand-over of a component gets a new
         * one, so an unbind and rebind never looks like the old ownership, even when the solver reuses the
         * released handle.
         */
        int64 Generation = 0;
        /** Chaos simulated the component before the hand-over. */
        bool bWasSimulatingPhysics = false;
    };

    /**
     * Who owned a bound body when a write-back was prepared. The body table and the solver slot can change
     * while a component is moved (delegates run), so a prepared write is only valid while the same owner
     * still holds the body: same component, binding, backend, handle, hand-over generation and solver object.
     */
    struct FBodyIdentity
    {
        TObjectKey<UPrimitiveComponent> ComponentKey;
        TObjectKey<UUBotPhysicsBindingComponent> BindingKey;
        FName Backend;
        int32 Handle = INDEX_NONE;
        /** FBodySlot::Generation of the hand-over the state was read for. */
        int64 Generation = 0;
        TWeakPtr<IUBotPhysicsSolver> Solver;
    };

    /**
     * One component of the world as the scene sees it, whoever simulates it. The shapes are extracted once; the
     * record ends (and a new one with a new id starts) when the kind, the body setup object or the world scale
     * changes, and when the component stops being part of the scene.
     */
    struct FSceneRecord
    {
        /** From NextSceneBodyId; names this record and nothing else. */
        int64 Id = INDEX_NONE;
        EUBotPhysicsSceneBodyKind Kind = EUBotPhysicsSceneBodyKind::Static;
        TWeakObjectPtr<UPrimitiveComponent> Component;
        TWeakObjectPtr<UBodySetup> BodySetup;
        /** World scale of the component when the shapes were extracted. */
        FVector WorldScale = FVector::OneVector;
        TArray<FUBotPhysicsShape> Shapes;
        float Friction = 0.0f;
        float Restitution = 0.0f;
    };

    /** Records a failed bind in Binding, reports it, copies Error to OutError and returns false. */
    static bool FailBind(UUBotPhysicsBindingComponent& Binding, EUBotPhysicsBindingStatus Status, const FText& Error, FText& OutError);

    void RegisterTickFunction(UWorld& InWorld);
    FSolverSlot* FindSolverSlot(FName Backend);
    const FSolverSlot* FindSolverSlot(FName Backend) const;
    const FBodySlot* FindBodySlot(const UPrimitiveComponent* Component) const;

    /** True while the body table still maps Identity's component to the binding, backend, handle and generation of Identity and the slot still holds its solver. */
    bool IsBodyOwnedBy(const FBodyIdentity& Identity) const;

    /** True while Factory is the registered factory of Backend and the slot of Backend still holds Solver. */
    bool IsBackendCurrent(FName Backend, const TSharedPtr<IUBotPhysicsBackendFactory>& Factory, const TSharedPtr<IUBotPhysicsSolver>& Solver) const;

    /**
     * Index of the slot of Backend in Solvers, creating its solver with Factory on first use (a slot whose
     * solver was destroyed is reused). The slot is marked while Factory->CreateSolver runs, and a call that
     * comes in meanwhile fails with Rejected. The new solver is put into the slot only if the slot is still
     * empty; otherwise it is dropped and the slot's is used. The body count of the slot is not touched.
     * INDEX_NONE with OutError and OutStatus (what the binding becomes) set when that fails: BackendMissing
     * when the factory was unregistered or replaced while it created the solver, else Rejected.
     */
    int32 FindOrCreateSolverSlotIndex(FName Backend, const TSharedPtr<IUBotPhysicsBackendFactory>& Factory, FText& OutError, EUBotPhysicsBindingStatus& OutStatus);

    /** Takes one body back from its solver and restores the component. */
    void ReleaseBody(TObjectKey<UPrimitiveComponent> Key);

    /** Releases the bodies whose component or binding no longer exists. */
    void ReleaseStaleBodies();

    /**
     * Takes every body of Backend back (as UnbindBodies does), sets their bindings to BackendMissing and reports
     * the runtime problem, and destroys the solver. The slot stays, so the dropped step count survives, and the
     * next solver of that backend reuses it.
     */
    void ReleaseBackend(FName Backend);

    /** ReleaseBackend for every backend whose registered factory is gone or is another object than the one that made its solver. */
    void ReleaseUnregisteredBackends();

    /** Handler added with FUBotExtensionRegistry::AddExtensionsChangedHandler; may run on any thread and defers the work to the game thread. */
    static void HandleExtensionsChanged(const TWeakObjectPtr<UUBotPhysicsSubsystem>& WeakSubsystem, FName PointName);

    /** Writes the state of every bound body back to its component, attach parents first. */
    void WriteBackPoses();

    /** True when some solver holds bodies and declares a scene capability. */
    bool IsSceneNeeded() const;

    /** Scans the world's components and brings SceneRecords up to date; counts what could not be exported. */
    void RefreshSceneRecords();

    /** Adds to OutKeys the components bound to Backend and every primitive component attached (directly or further down) to them. */
    void CollectOwnComponents(FName Backend, TSet<TObjectKey<UPrimitiveComponent>>& OutKeys) const;

    /**
     * Sends the solver of the slot what changed in its scene since its last update and remembers what it
     * knows. The slot is not referenced across the call into the backend. False when the slot lost its
     * solver while UpdateScene ran.
     */
    bool DeliverScene(int32 SlotIndex, const TSharedPtr<IUBotPhysicsSolver>& Solver, double SimTimeSeconds);

    FUBotPhysicsTickFunction TickFunction;
    TArray<FSolverSlot> Solvers;
    /** Keyed by object key, not by address: a destroyed component's address can be reused by another object. */
    TMap<TObjectKey<UPrimitiveComponent>, FBodySlot> Bodies;
    FDelegateHandle ExtensionsChangedHandle;
    /** Last hand-over generation given out; counts up for the life of the subsystem, never reused. */
    int64 NextBodyGeneration = 0;
    /** The components that are part of the scene, keyed by object key; kept up to date by RefreshSceneRecords. */
    TMap<TObjectKey<UPrimitiveComponent>, FSceneRecord> SceneRecords;
    /** Last scene body id given out; counts up for the life of the subsystem, never reused. */
    int64 NextSceneBodyId = 0;
    /** Components with physics collision that the last scan could not export. */
    int32 SceneSkippedComponents = 0;
    int32 MaxStepsPerFrame = 1000;
};
