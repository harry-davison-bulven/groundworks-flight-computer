param([string]$KspRoot='C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program')
$ErrorActionPreference='Stop'
$managed=Join-Path $KspRoot 'KSP_x64_Data\Managed'
$compiler=Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
$refs=@("/reference:$managed\Assembly-CSharp.dll","/reference:$managed\UnityEngine.dll","/reference:$managed\UnityEngine.CoreModule.dll","/reference:$managed\UnityEngine.UI.dll","/reference:$managed\UnityEngine.PhysicsModule.dll","/reference:$managed\UnityEngine.ParticleSystemModule.dll","/reference:$managed\UnityEngine.IMGUIModule.dll","/reference:$managed\UnityEngine.InputLegacyModule.dll","/reference:$managed\UnityEngine.AnimationModule.dll","/reference:$managed\UnityEngine.TextRenderingModule.dll")
$out=Join-Path $PSScriptRoot 'GameData\GroundworksFlightComputer\Plugins'
New-Item -ItemType Directory -Force $out | Out-Null
$sources=@((Join-Path $PSScriptRoot 'GroundworksFlightComputer.cs'),(Join-Path $PSScriptRoot 'GroundworksAutopilot.cs'),(Join-Path $PSScriptRoot 'GroundworksFlightTools.cs'))
& $compiler /nologo /target:library "/out:$out\GroundworksFlightComputer.dll" @refs @sources
if($LASTEXITCODE -ne 0){throw "Build failed"}
