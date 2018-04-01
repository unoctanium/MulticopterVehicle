// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;

public class QCTestProject : ModuleRules
{
	public QCTestProject(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { 
			"Core", "CoreUObject", "Engine", "InputCore", "PhysX", "APEX", "Sockets", "Networking", "UMG" 
		});

		PrivateDependencyModuleNames.AddRange(new string[] { "HeadMountedDisplay" });
        
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
