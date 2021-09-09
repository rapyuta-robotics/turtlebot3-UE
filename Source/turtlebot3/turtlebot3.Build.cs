// Copyright (c) Rapyuta Robotics Co., Ltd.

using UnrealBuildTool;

public class turtlebot3 : ModuleRules
{
    public turtlebot3(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "rclUE", "UE_rapyuta_assets" });

        PrivateDependencyModuleNames.AddRange(new string[] { });
    }
}
