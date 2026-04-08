using System;
using System.IO;
using UnrealBuildTool;

public class CarlaRGL : ModuleRules
{
    public CarlaRGL(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        bEnableExceptions = true;
        bUseRTTI = true;

        PrivateDependencyModuleNames.AddRange(new string[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "RenderCore",
            "RHI",
            "PhysicsCore",
            "Chaos",
            "Carla"
        });

        string PluginDirectory = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", ".."));
        string DefsPath = Path.Combine(PluginDirectory, "Definitions.def");
        bool EnableRGL = false;

        if (File.Exists(DefsPath))
        {
            foreach (var Def in File.ReadAllText(DefsPath).Split(';'))
            {
                if (Def.Trim() == "WITH_RGL")
                {
                    EnableRGL = true;
                    break;
                }
            }
        }

        if (EnableRGL)
        {
            PrivateDefinitions.Add("WITH_RGL");

            string BinLinux = Path.Combine(PluginDirectory, "Binaries", "Linux");
            string RglSo = Path.Combine(BinLinux, "libRobotecGPULidar.so");
            if (File.Exists(RglSo))
            {
                PublicAdditionalLibraries.Add(RglSo);
                RuntimeDependencies.Add(RglSo);
                PublicDelayLoadDLLs.Add(RglSo);
                Console.WriteLine("CarlaRGL: libRobotecGPULidar.so linked.");
            }
            else
            {
                Console.WriteLine("CarlaRGL WARNING: libRobotecGPULidar.so not found at " + RglSo);
            }

            string IncsPath = Path.Combine(PluginDirectory, "Includes.def");
            if (File.Exists(IncsPath))
            {
                foreach (var Inc in File.ReadAllText(IncsPath).Split(';'))
                {
                    var t = Inc.Trim();
                    if (t.Length > 0 && t.Contains("RobotecGPULidar"))
                    {
                        PrivateIncludePaths.Add(t);
                    }
                }
            }
        }
    }
}
