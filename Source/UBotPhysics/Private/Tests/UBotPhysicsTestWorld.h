#pragma once

#if WITH_DEV_AUTOMATION_TESTS

#include "CoreMinimal.h"
#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerState.h"
#include "GameFramework/WorldSettings.h"
#include "UBotPhysicsSubsystem.h"

namespace UBotPhysicsTests
{
    constexpr float FrameSeconds = 1.0f / 60.0f;

    // Transient game world, initialized for play and ticked by hand; torn down when the scope ends. Unless told
    // not to, it tells the physics subsystem that play began, as UWorld::BeginPlay does for a real world, so
    // that its tick function runs. Same pattern as the uBotAerial tests.
    class FScopedPhysicsTestWorld
    {
    public:
        UE_NONCOPYABLE(FScopedPhysicsTestWorld);

        explicit FScopedPhysicsTestWorld(bool bBeginPlay = true)
            : World(UWorld::CreateWorld(EWorldType::Game, false))
            , CachedFrameCounter(GFrameCounter)
        {
            if (World && GEngine)
            {
                World->AddToRoot();
                GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
                World->InitializeActorsForPlay(FURL());
                Subsystem = World->GetSubsystem<UUBotPhysicsSubsystem>();
                if (bBeginPlay)
                {
                    BeginPlay();
                }
            }
        }

        ~FScopedPhysicsTestWorld()
        {
            if (!World)
            {
                return;
            }
            if (Pauser)
            {
                SetPaused(false);
            }
            if (World->AreActorsInitialized())
            {
                for (TActorIterator<AActor> It(World); It; ++It)
                {
                    It->RouteEndPlay(EEndPlayReason::LevelTransition);
                }
            }
            if (GEngine)
            {
                GEngine->DestroyWorldContext(World);
            }
            World->DestroyWorld(false);
            World->RemoveFromRoot();
            GFrameCounter = CachedFrameCounter;
        }

        UWorld* Get() const { return World; }

        UUBotPhysicsSubsystem* GetSubsystem() const { return Subsystem; }

        /** What UWorld::BeginPlay does for the subsystem; the constructor calls it unless told not to. */
        void BeginPlay() const
        {
            if (Subsystem)
            {
                Subsystem->OnWorldBeginPlay(*World);
            }
        }

        double Now() const { return World->GetTimeSeconds(); }

        void Tick(float DeltaSeconds) const
        {
            World->Tick(LEVELTICK_All, DeltaSeconds);
            // A tick function runs at most once per GFrameCounter value.
            ++GFrameCounter;
        }

        /** Ticks Frames frames of DeltaSeconds. */
        void RunFrames(int32 Frames, float DeltaSeconds = FrameSeconds) const
        {
            for (int32 Frame = 0; Frame < Frames; ++Frame)
            {
                Tick(DeltaSeconds);
            }
        }

        /**
         * An actor whose root is a box component named "Box" (50 cm cube) at LocationCm. With bSimulate it is a Chaos
         * physics body; without, it has no collision at all. The box is registered but the actor has not begun play.
         */
        AActor* SpawnBoxActor(const FVector& LocationCm, bool bSimulate = true, EComponentMobility::Type Mobility = EComponentMobility::Movable) const
        {
            AActor* Actor = World->SpawnActor<AActor>();
            UBoxComponent* Box = NewObject<UBoxComponent>(Actor, TEXT("Box"));
            Box->SetBoxExtent(FVector(25.0), false);
            Box->SetMobility(Mobility);
            Box->SetCollisionProfileName(bSimulate ? UCollisionProfile::PhysicsActor_ProfileName : UCollisionProfile::NoCollision_ProfileName);
            Actor->SetRootComponent(Box);
            Box->SetWorldLocation(LocationCm);
            Box->RegisterComponent();
            if (bSimulate)
            {
                Box->SetSimulatePhysics(true);
            }
            return Actor;
        }

        bool SetPaused(bool bPaused)
        {
            AWorldSettings* Settings = World ? World->GetWorldSettings(false, false) : nullptr;
            if (!Settings)
            {
                return false;
            }
            if (bPaused && !Pauser)
            {
                Pauser = World->SpawnActor<APlayerState>();
            }
            Settings->SetPauserPlayerState(bPaused ? Pauser : nullptr);
            return World->IsPaused() == bPaused;
        }

    private:
        UWorld* World = nullptr;
        UUBotPhysicsSubsystem* Subsystem = nullptr;
        APlayerState* Pauser = nullptr;
        uint64 CachedFrameCounter = 0;
    };
}

#endif
