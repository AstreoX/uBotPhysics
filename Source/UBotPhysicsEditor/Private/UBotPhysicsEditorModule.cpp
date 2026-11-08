#include "Modules/ModuleManager.h"

#include "Editor.h"
#include "UBotPhysicsModelActorFactory.h"

class FUBotPhysicsEditorModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        if (GEditor != nullptr)
        {
            Factory = NewObject<UUBotPhysicsModelActorFactory>();
            GEditor->ActorFactories.Add(Factory);
        }
    }

    virtual void ShutdownModule() override
    {
        if (GEditor != nullptr && Factory != nullptr) { GEditor->ActorFactories.Remove(Factory); }
        Factory = nullptr;
    }
private:
    UUBotPhysicsModelActorFactory* Factory = nullptr;
};

IMPLEMENT_MODULE(FUBotPhysicsEditorModule, UBotPhysicsEditor)
