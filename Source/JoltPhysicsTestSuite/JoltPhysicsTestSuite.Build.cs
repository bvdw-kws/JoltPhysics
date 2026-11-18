// Copyright (C) 2024 Van de Walle Bastien
// SPDX-License-Identifier: MIT

using UnrealBuildTool;

public class JoltPhysicsTestSuite : ModuleRules
{
	public JoltPhysicsTestSuite(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// Same reason as JoltPhysicsRuntime: the tests do a lot of Jolt work and must not leak
		// JPH::uint64/int64 across translation units of a unity build.
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"JoltPhysics",
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
		});
	}
}
