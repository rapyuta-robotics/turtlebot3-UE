// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

using UnrealBuildTool;

public class turtlebot3 : ModuleRules
{
    public turtlebot3(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "rclUE", "RapyutaSimulationPlugins", "RapyutaSimRobotImporter" });

        PrivateDependencyModuleNames.AddRange(new string[] { });
    }
}
