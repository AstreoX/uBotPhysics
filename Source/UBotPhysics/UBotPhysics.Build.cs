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

        // Collision shapes of the scene: body setups (PhysicsCore) and the triangle meshes Chaos holds.
        PrivateDependencyModuleNames.AddRange(new[]
        {
            "ChaosCore",
            "Chaos",
            "PhysicsCore"
        });
    }
}
