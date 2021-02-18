// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class turtlebot3 : ModuleRules
{
	public turtlebot3(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "rclUE" });

		PrivateDependencyModuleNames.AddRange(new string[] { });
	}
}
