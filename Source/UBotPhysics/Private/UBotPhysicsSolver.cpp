#include "UBotPhysicsBackend.h"

bool IUBotPhysicsSolver::AddModel(const FUBotPhysicsModelInstanceDesc& Desc, FUBotPhysicsModelInstanceResult& OutResult, FText& OutError)
{
    OutResult.Bodies.Reset();
    if (!Desc.Model.Validate(OutError)) { return false; }
    if (Desc.InstanceId < 0 || BodyOnlyModels.Contains(Desc.InstanceId) || Desc.Model.Extension != nullptr || !Desc.Model.Drives.IsEmpty())
    {
        OutError = INVTEXT("This body-only backend cannot register this model instance or its native extension/drives.");
        return false;
    }
    for (const FUBotPhysicsModelJoint& Joint : Desc.Model.Joints)
    {
        if (Joint.Type != EUBotPhysicsJointType::Free)
        {
            OutError = INVTEXT("This body-only backend does not support articulated models.");
            return false;
        }
    }
    for (const FUBotPhysicsModelBody& Body : Desc.Model.Bodies)
    {
        const int32 FreeJoints = Desc.Model.Joints.FilterByPredicate([&Body](const FUBotPhysicsModelJoint& Joint)
        {
            return Joint.BodyId.Equals(Body.Id, ESearchCase::CaseSensitive) && Joint.Type == EUBotPhysicsJointType::Free;
        }).Num();
        if (!Body.ParentId.IsEmpty() || Body.bKinematic || FreeJoints != 1)
        {
            OutError = INVTEXT("This body-only backend only accepts independent free rigid bodies.");
            return false;
        }
    }
    if (Desc.BodyBindings.Num() != Desc.Model.Bodies.Num())
    {
        OutError = INVTEXT("This body-only backend requires a display binding for every model body.");
        return false;
    }
    TArray<int32> Handles;
    for (const FUBotPhysicsModelBodyBindingDesc& Body : Desc.BodyBindings)
    {
        const int32 Handle = AddBody(Body.Desc, OutError);
        if (Handle >= 0)
        {
            Handles.Add(Handle);
            OutResult.Bodies.Add({ Body.BodyId, Handle });
        }
        if (Handle < 0 || (Desc.IsCurrent && !Desc.IsCurrent()))
        {
            for (int32 Index = Handles.Num() - 1; Index >= 0; --Index)
            {
                RemoveBody(Handles[Index]);
            }
            OutResult.Bodies.Reset();
            return false;
        }
    }
    BodyOnlyModels.Add(Desc.InstanceId, MoveTemp(Handles));
    return true;
}

bool IUBotPhysicsSolver::RemoveModel(int64 InstanceId, FText& OutError)
{
    TArray<int32> Handles;
    if (!BodyOnlyModels.RemoveAndCopyValue(InstanceId, Handles))
    {
        OutError = INVTEXT("Unknown physics model instance.");
        return false;
    }
    for (int32 Index = Handles.Num() - 1; Index >= 0; --Index)
    {
        RemoveBody(Handles[Index]);
    }
    return true;
}
