#pragma once

#include "CoreMinimal.h"

class UActorComponent;

namespace UBot::Physics::Private
{
    /**
     * The id of the runtime problem of Binding: "physics.binding." plus the binding component's path name, so
     * bindings in different worlds and several bindings on one actor never share an id.
     */
    FString MakeBindingProblemId(const UActorComponent& Binding);

    /**
     * Reports a failed Bind to the uBot runtime status as a problem (id see MakeBindingProblemId, code
     * "physicsBindingFailed", severity Warning, params "actor" and "backend") and logs it. Replaces the earlier
     * problem of the same binding.
     */
    void ReportBindingFailure(const UActorComponent& Binding, FName Backend, const FText& Error);

    /** Withdraws the problem ReportBindingFailure raised for Binding. Does nothing if there is none. */
    void ClearBindingFailure(const UActorComponent& Binding);
}
