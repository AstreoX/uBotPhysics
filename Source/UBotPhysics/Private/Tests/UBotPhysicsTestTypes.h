#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "UBotBodyState.h"
#include "UBotExtensionRegistry.h"
#include "UBotPhysics.h"
#include "UBotPhysicsBackend.h"
#include "UBotPhysicsBindingComponent.h"
#include "UBotPhysicsTestTypes.generated.h"

// Test-only binding: the test backend moves every body it hands over at these rates.
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotPhysicsTestBinding : public UUBotPhysicsBindingComponent
{
    GENERATED_BODY()

public:
    /** World frame, m/s. */
    UPROPERTY()
    FVector VelocityMetersPerSecond = FVector::ZeroVector;

    /** About world Z, rad/s (+Z yaws right). */
    UPROPERTY()
    double YawRateRadiansPerSecond = 0.0;
};

// Test-only body state source that claims one component, to stand for another mover such as a vehicle's own integrator.
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotPhysicsTestStateSource : public UActorComponent, public IUBotBodyStateSource
{
    GENERATED_BODY()

public:
    UPROPERTY()
    TObjectPtr<USceneComponent> ProvidedBody;

    virtual bool ProvidesBodyState(const USceneComponent* Body) const override
    {
        return Body != nullptr && Body == ProvidedBody;
    }

    virtual bool GetBodyState(const USceneComponent* Body, FUBotBodyState& OutState) const override
    {
        if (!ProvidesBodyState(Body))
        {
            return false;
        }
        OutState = FUBotBodyState();
        OutState.Source = TEXT("UBotPhysicsTestStateSource");
        return true;
    }
};

// The test backend. Plain C++, but it lives here with the reflected types it uses: UHT does not allow
// WITH_DEV_AUTOMATION_TESTS guards in this header, so everything is compiled in every configuration.
namespace UBotPhysicsTests
{
    /** Name of the test backend, as a binding's Backend. */
    inline const FName BackendName(TEXT("UBotPhysicsTest"));

    /** Package the test backend is registered for; one call to UnregisterAllFromPackage removes it. */
    inline const FName TestPackage(TEXT("UBotPhysicsTests"));

    /** The test solver refuses bodies with this name. */
    inline const FName RejectBodyName(TEXT("Reject"));

    /**
     * Solver of the test backend: every body moves on a straight line at the velocity (and yaws at the rate) of
     * the test binding that handed it over; a body without such a binding stands still. Records what it was asked.
     */
    class FUBotPhysicsTestSolver final : public IUBotPhysicsSolver
    {
    public:
        explicit FUBotPhysicsTestSolver(double InStepSeconds)
            : StepSeconds(InStepSeconds)
        {
        }

        struct FBody
        {
            FName Name;
            FVector OriginMeters = FVector::ZeroVector;
            FQuat OriginRotation = FQuat::Identity;
            FVector VelocityMetersPerSecond = FVector::ZeroVector;
            double YawRateRadiansPerSecond = 0.0;
            int32 StepsTaken = 0;
        };

        virtual int32 AddBody(const FUBotPhysicsBodyDesc& Desc, FText& OutError) override
        {
            if (Desc.BodyName == RejectBodyName)
            {
                OutError = INVTEXT("the test solver refuses bodies named Reject");
                return INDEX_NONE;
            }

            ++AddCalls;
            FBody Body;
            Body.Name = Desc.BodyName;
            Body.OriginMeters = Desc.InitialWorldTransform.GetLocation() / 100.0;
            Body.OriginRotation = Desc.InitialWorldTransform.GetRotation();
            if (const UUBotPhysicsTestBinding* Binding = Cast<UUBotPhysicsTestBinding>(Desc.Binding.Get()))
            {
                Body.VelocityMetersPerSecond = Binding->VelocityMetersPerSecond;
                Body.YawRateRadiansPerSecond = Binding->YawRateRadiansPerSecond;
            }
            // The component must already be kinematic: the subsystem hands it over that way.
            if (const UPrimitiveComponent* Component = Desc.Component.Get())
            {
                bAllKinematicOnAdd &= !Component->IsSimulatingPhysics();
            }

            int32 Handle = INDEX_NONE;
            if (bReuseHandles && !ReleasedHandles.IsEmpty())
            {
                // The handle of the body that left last, as a real backend with a free list would do.
                Handle = ReleasedHandles.Pop();
            }
            else
            {
                Handle = NextHandle++;
            }
            IssuedHandles.Add(Handle);
            Bodies.Add(Handle, Body);
            if (OnBodyAdded)
            {
                OnBodyAdded(AddCalls);
            }
            return Handle;
        }

        virtual void RemoveBody(int32 BodyHandle) override
        {
            ++RemoveCalls;
            if (Bodies.Remove(BodyHandle) > 0 && bReuseHandles)
            {
                ReleasedHandles.Add(BodyHandle);
            }
        }

        virtual double GetStepSeconds() const override
        {
            return StepSeconds;
        }

        virtual void Step(double StepEndTimeSeconds) override
        {
            StepEndTimes.Add(StepEndTimeSeconds);
            for (TPair<int32, FBody>& Pair : Bodies)
            {
                ++Pair.Value.StepsTaken;
            }
        }

        virtual bool GetBodyState(int32 BodyHandle, FUBotBodyState& OutState) const override
        {
            const FBody* Body = Bodies.Find(BodyHandle);
            if (StepEndTimes.IsEmpty() || Body == nullptr)
            {
                return false;
            }
            const double Elapsed = static_cast<double>(Body->StepsTaken) * StepSeconds;
            OutState.PositionMeters = Body->OriginMeters + Body->VelocityMetersPerSecond * Elapsed;
            OutState.Orientation = FQuat(FVector::UpVector, Body->YawRateRadiansPerSecond * Elapsed) * Body->OriginRotation;
            OutState.LinearVelocityMetersPerSecond = Body->VelocityMetersPerSecond;
            OutState.AngularVelocityRadiansPerSecond = FVector(0.0, 0.0, Body->YawRateRadiansPerSecond);
            return true;
        }

        const FBody* FindBody(int32 BodyHandle) const
        {
            return Bodies.Find(BodyHandle);
        }

        int32 GetBodyCount() const
        {
            return Bodies.Num();
        }

        double StepSeconds = 0.01;
        TArray<double> StepEndTimes;
        int32 AddCalls = 0;
        int32 RemoveCalls = 0;
        /** False if a body was ever handed over while Chaos still simulated its component. */
        bool bAllKinematicOnAdd = true;
        /** Called inside AddBody once the body is accepted, with the number of bodies accepted so far (this one included). */
        TFunction<void(int32)> OnBodyAdded;
        /** Hand out the handle of a released body again (the one that left last) before it counts up to a new number. */
        bool bReuseHandles = false;
        /** Every handle AddBody gave out, in order. */
        TArray<int32> IssuedHandles;

    private:
        TMap<int32, FBody> Bodies;
        TArray<int32> ReleasedHandles;
        int32 NextHandle = 0;
    };

    /** Factory of the test backend. */
    class FUBotPhysicsTestBackendFactory final : public IUBotPhysicsBackendFactory
    {
    public:
        explicit FUBotPhysicsTestBackendFactory(double InStepSeconds = 0.01, FName InName = BackendName)
            : StepSeconds(InStepSeconds)
            , Name(InName)
        {
        }

        virtual FName GetExtensionName() const override
        {
            return Name;
        }

        virtual FUBotPhysicsCapabilities GetCapabilities() const override
        {
            FUBotPhysicsCapabilities Capabilities;
            Capabilities.bRigidBodies = true;
            Capabilities.bKinematicForeignBodies = true;
            Capabilities.bDeterministic = true;
            return Capabilities;
        }

        virtual TSharedPtr<IUBotPhysicsSolver> CreateSolver(UWorld& World) override
        {
            ++CreateCalls;
            if (bFailCreate)
            {
                return nullptr;
            }
            const TSharedRef<FUBotPhysicsTestSolver> Solver = MakeShared<FUBotPhysicsTestSolver>(StepSeconds);
            Solver->OnBodyAdded = OnBodyAdded;
            Solver->bReuseHandles = bReuseHandles;
            LastSolver = Solver;
            if (bKeepSolver)
            {
                KeptSolver = Solver;
            }
            // Runs the backend's own code inside CreateSolver, e.g. to unregister the factory from within.
            if (OnCreateSolver)
            {
                OnCreateSolver();
            }
            return Solver;
        }

        double StepSeconds = 0.01;
        FName Name;
        bool bFailCreate = false;
        int32 CreateCalls = 0;
        /** The solver made last; expires when the subsystem lets it go. */
        TWeakPtr<IUBotPhysicsSolver> LastSolver;
        /** Keep every solver alive in KeptSolver (the last one), so a test can inspect it after the subsystem dropped it. */
        bool bKeepSolver = false;
        TSharedPtr<FUBotPhysicsTestSolver> KeptSolver;
        /** Called inside CreateSolver, after the solver was made and before it is returned. */
        TFunction<void()> OnCreateSolver;
        /** Handed to every solver made afterwards as its OnBodyAdded. */
        TFunction<void(int32)> OnBodyAdded;
        /** Handed to every solver made afterwards as its bReuseHandles. */
        bool bReuseHandles = false;
    };

    /**
     * Registers a test backend for the scope; the registration is removed even when the test fails early.
     * Backends that live side by side need their own Package, because removing a package removes all of it.
     */
    class FScopedTestBackend
    {
    public:
        UE_NONCOPYABLE(FScopedTestBackend);

        explicit FScopedTestBackend(double StepSeconds = 0.01, FName Name = BackendName, FName InPackage = TestPackage)
            : Factory(MakeShared<FUBotPhysicsTestBackendFactory>(StepSeconds, Name))
            , Package(InPackage)
        {
            bRegistered = FUBotExtensionRegistry::Get().RegisterExtension(UBot::Physics::BackendExtensionPoint, Package, Factory);
        }

        ~FScopedTestBackend()
        {
            FUBotExtensionRegistry::Get().UnregisterAllFromPackage(Package);
        }

        TSharedRef<FUBotPhysicsTestBackendFactory> Factory;
        FName Package;
        bool bRegistered = false;
    };
}
