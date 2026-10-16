#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "UBotPhysicsTypes.h"
#include "UBotPhysicsLibrary.generated.h"

class UPrimitiveComponent;

/** Blueprint access to the uBot physics backends and to which backend simulates a component. */
UCLASS(meta = (DisplayName = "uBot Physics Library"))
class UBOTPHYSICS_API UUBotPhysicsLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    /** Names of the registered physics backends, in registration order. A binding's Backend is one of them. */
    UFUNCTION(BlueprintPure, Category = "uBot|Physics", meta = (DisplayName = "Get uBot Physics Backend Names", Keywords = "physics backend list solver"))
    static TArray<FName> GetPhysicsBackendNames();

    /** What Backend can simulate. False (and all capabilities off) if no backend has that name. */
    UFUNCTION(BlueprintPure, Category = "uBot|Physics", meta = (DisplayName = "Get uBot Physics Backend Capabilities", Keywords = "physics backend capabilities features"))
    static bool GetPhysicsBackendCapabilities(FName Backend, FUBotPhysicsCapabilities& Capabilities);

    /** The backend that simulates Component; None when Chaos (the engine's physics) does. */
    UFUNCTION(BlueprintPure, Category = "uBot|Physics", meta = (DisplayName = "Get Body Physics Backend", Keywords = "physics backend chaos owner"))
    static FName GetBodyPhysicsBackend(const UPrimitiveComponent* Component);

    /** Steps the backend skipped in this world because one frame needed more than the per-frame limit. */
    UFUNCTION(BlueprintPure, Category = "uBot|Physics", meta = (WorldContext = "WorldContextObject", DisplayName = "Get Dropped Physics Steps", Keywords = "physics backend dropped skipped steps"))
    static int64 GetDroppedPhysicsSteps(const UObject* WorldContextObject, FName Backend);

    /**
     * What Backend currently sees of this world's scene: the static and kinematic colliders it was last sent, their
     * shapes, and how many components with physics collision could not be exported. False (and all zero) if the
     * backend has no solver in this world.
     */
    UFUNCTION(BlueprintPure, Category = "uBot|Physics", meta = (WorldContext = "WorldContextObject", DisplayName = "Get Physics Scene Stats", Keywords = "scene collider static kinematic mocap geometry"))
    static bool GetPhysicsSceneStats(const UObject* WorldContextObject, FName Backend, FUBotPhysicsSceneStats& Stats);
};
