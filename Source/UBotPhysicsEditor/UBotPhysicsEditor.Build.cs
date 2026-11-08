using UnrealBuildTool;

public class UBotPhysicsEditor : ModuleRules
{
    public UBotPhysicsEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PrivateDependencyModuleNames.AddRange(new[]
        {
            "Core", "CoreUObject", "Engine", "UnrealEd", "AssetRegistry", "UBotPhysics"
        });
    }
}
