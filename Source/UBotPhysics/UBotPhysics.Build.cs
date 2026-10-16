using UnrealBuildTool;

public class UBotPhysics : ModuleRules
{
    public UBotPhysics(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "UBotCore"
        });
    }
}
