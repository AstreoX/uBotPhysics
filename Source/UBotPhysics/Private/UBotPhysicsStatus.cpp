#include "UBotPhysicsStatus.h"

#include "Components/ActorComponent.h"
#include "GameFramework/Actor.h"
#include "UBotPhysics.h"
#include "UBotRuntimeStatus.h"

namespace UBot::Physics::Private
{
    FString MakeBindingProblemId(const UActorComponent& Binding)
    {
        return FString::Printf(TEXT("physics.binding.%s"), *Binding.GetPathName());
    }

    void ReportBindingFailure(const UActorComponent& Binding, FName Backend, const FText& Error)
    {
        const FString ActorName = GetNameSafe(Binding.GetOwner());
        UE_LOG(LogUBotPhysics, Warning, TEXT("Physics binding of %s (backend '%s') failed: %s"),
            *ActorName, *Backend.ToString(), *Error.ToString());

        FUBotRuntimeProblem Problem;
        Problem.Id = MakeBindingProblemId(Binding);
        Problem.Package = PackageName.ToString();
        Problem.Severity = EUBotRuntimeSeverity::Warning;
        Problem.Code = TEXT("physicsBindingFailed");
        Problem.Params.Emplace(TEXT("actor"), ActorName);
        Problem.Params.Emplace(TEXT("backend"), Backend.ToString());
        Problem.Message = Error;
        FUBotRuntimeStatus::Get().ReportProblem(Problem);
    }

    void ClearBindingFailure(const UActorComponent& Binding)
    {
        FUBotRuntimeStatus::Get().ClearProblem(MakeBindingProblemId(Binding));
    }

    void ReportBackendFailure(const UObject& Context, FName Backend, const FText& Error)
    {
        UE_LOG(LogUBotPhysics, Warning, TEXT("Physics backend '%s' suspended: %s"), *Backend.ToString(), *Error.ToString());
        FUBotRuntimeProblem Problem;
        Problem.Id = FString::Printf(TEXT("physics.backend.%s.%s"), *Context.GetPathName(), *Backend.ToString());
        Problem.Package = PackageName.ToString();
        Problem.Severity = EUBotRuntimeSeverity::Warning;
        Problem.Code = TEXT("physicsBackendSuspended");
        Problem.Params.Emplace(TEXT("backend"), Backend.ToString());
        Problem.Message = Error;
        FUBotRuntimeStatus::Get().ReportProblem(Problem);
    }

    void ClearBackendFailure(const UObject& Context, FName Backend)
    {
        FUBotRuntimeStatus::Get().ClearProblem(FString::Printf(TEXT("physics.backend.%s.%s"), *Context.GetPathName(), *Backend.ToString()));
    }
}
