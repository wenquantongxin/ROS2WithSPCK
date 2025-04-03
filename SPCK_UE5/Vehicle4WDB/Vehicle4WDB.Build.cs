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
            "UMG",        // ʹ�� UUserWidget
            "SlateCore",  // �����Ҫ
            "Json",
            "JsonUtilities"  // �����֧��JSON����
        });
    }
}
