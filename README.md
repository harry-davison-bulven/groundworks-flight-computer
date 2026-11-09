# Groundworks Flight Computer

Standalone attitude, launch, maneuver and powered-landing guidance with flight tools.

Version **0.1.0-alpha.1**, experimental alpha for **KSP 1.12.5**. Tested on Windows. MIT license. No parts pack is required.

A stock-toolbar flight computer for connected, powered controllable vessels. It provides stock SAS attitude modes, a one-shot apoapsis cutoff, original launch-to-orbit guidance, maneuver-node execution, and powered descent/landing guidance. Tools include flight profiles, surface bookmarks, a local flight log and orbit-node planning. Open the Groundworks toolbar button in flight. X aborts active guidance and cuts throttle while not editing an input. Check staging, controls, thrust, propellant and the target before engaging automation.

No original parts, MechJeb, external autopilot or third-party plugin is required. The optional original-shuttle adapter is dormant without that hardware; this package supplies no winged shuttle recovery controller. Precision runway recovery, rendezvous/docking and guaranteed safe autonomous missions are not implemented. Powered landing depends on sufficient thrust, fuel and attitude control; guidance is experimental and does not guarantee recovery.

## Install

Download `GroundworksFlightComputer-0.1.0-alpha.1.zip` from Releases, then merge its GameData folder into KSP with the game closed. Do not install the source archive as a mod. Restart KSP. The two standalone plugins may be installed independently or together. Neither requires the other.

To uninstall, close KSP and remove only `GameData/GroundworksFlightComputer`. Keep a backup of your saves. No stock files or save files need editing. Plugin settings or module records can remain in existing saves after removal.

## Validation and limitations

`NATIVE-TESTS.txt` records controlled stock-only integration tests using an uncrewed probe and three stock tanks. Fuel tests cover isolated depletion, fill-dependent flow, ignition, negative cases, visuals, saved module state, native rupture transfer and a physical external contact. Flight-computer checks cover stock-probe readiness, SAS hold, abort and rejection of a grounded tank bench. Model/math unit checks are included separately. Prior automatic flight development used other craft; this alpha does not claim a fresh end-to-end stock launch/landing qualification or certification of arbitrary mod packs.

## Source and build

Production C# source is in this repository root. `Build.ps1 -KspRoot <path>` builds against your locally installed KSP managed assemblies using the Windows .NET Framework C# compiler. The KSP SDK/game DLLs and test harness binaries are not redistributed. Test source files are excluded from the production build. `GroundworksFlightComputer-Source-0.1.0-alpha.1.zip` is also attached to the release.

## Distribution status

GitHub is the canonical release source. CurseForge and CKAN listings require separate submissions; metadata in this repository does not itself mean either listing is live.
