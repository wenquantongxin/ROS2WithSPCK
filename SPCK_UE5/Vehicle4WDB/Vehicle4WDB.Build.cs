// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class Vehicle4WDB : ModuleRules
{
	public Vehicle4WDB(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        //PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput" });
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core",
            "CoreUObject",
            "Engine",
            "InputCore",
            "EnhancedInput",
            "Networking",
            "Sockets",
            "UMG",        // 使用 UUserWidget
            "SlateCore",  // 如果需要
            "Json",
            "JsonUtilities"  // 添加以支持JSON处理
        });
    }
}
