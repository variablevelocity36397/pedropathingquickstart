# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

This is a fork of the official FIRST Tech Challenge (FTC) `FtcRobotController` SDK (currently the DECODE 2025-2026 season SDK), used as a quickstart for teams running [PedroPathing](https://pedropathing.com) — a path-following/motion-profiling library for FTC robots. It is an Android Studio (Gradle) project, not a general-purpose application: the build produces an APK that runs on an FTC Robot Controller device, and most "testing" happens on physical hardware, not in a desktop test runner.

Two Gradle modules:
- `FtcRobotController` — the stock FTC SDK app module (vendored, do not modify unless updating the SDK itself). Contains the large `external/samples` tree of official example OpModes.
- `TeamCode` — where team/robot code actually lives. Depends on `FtcRobotController`. This is almost the only module you should be editing.

## Build commands

Standard Android Gradle wrapper. Building requires the Android SDK to be configured (see `local.properties`); a full build will not succeed without it.

```
./gradlew build                        # build both modules
./gradlew :TeamCode:assembleDebug      # build just TeamCode (fastest sanity check)
./gradlew :TeamCode:compileDebugSources # compile-only check, faster than a full assemble
```

There is no meaningful unit/instrumented test suite in this repo (no `src/test` or `src/androidTest` sources exist beyond the boilerplate Gradle tasks) — correctness is verified by deploying to hardware, not by running `./gradlew test`.

## Architecture

### TeamCode / pedroPathing package

All current team code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/`:

- **`Constants.java`** — the single source of truth for robot tuning constants. `followerConstants` (a `FollowerConstants`) and `pathConstraints` (a `PathConstraints`) are configured here, and `createFollower(hardwareMap)` builds a `Follower` via `FollowerBuilder`. Any OpMode that wants to drive the robot via PedroPathing calls `Constants.createFollower(...)` to get a configured `Follower`. When tuning a new robot, this is the file that gets edited with the values produced by the tuners below.

- **`Tuning.java`** — a single large file containing a `SelectableOpMode` (`Tuning`, `@TeleOp` group "Pedro Pathing") plus ~20 package-private `OpMode` classes, one per tuning routine, all nested in the same file. The `Tuning` class builds a menu (via `SelectableOpMode`'s `s.folder(...)`/`.add(name, Ctor::new)` DSL) organized into folders: **Localization**, **Automatic**, **Manual**, **Tests**, **Swerve**. Each menu entry maps to one of the OpMode classes defined later in the file (e.g. `ForwardTuner`, `LateralVelocityTuner`, `PredictiveBrakingTuner`, `SwerveOffsetsTest`). These OpModes are run directly on the Driver Station to empirically derive values (ticks-to-inches multipliers, PIDF constants, zero-power decelerations, swerve encoder min/max, etc.) that a team then copies into `Constants.java`.
  - A shared static `follower` field is built once via `Constants.createFollower(hardwareMap)` and reused across tuner selections.
  - `Drawing` (bottom of the file) wraps the Panels/`FieldManager` dashboard drawing API (`bylazar` Panels) for visualizing robot pose/path/history during tuning.
  - When adding a new tuner: add a nested `OpMode` class in `Tuning.java` and register it in the appropriate `s.folder(...)` block in the `Tuning` constructor — that's the whole integration point.

### Dependencies (`build.dependencies.gradle`)

This is where the PedroPathing/Panels library versions are pinned:
- `com.pedropathing:ftc` — the core path-following library (`Follower`, `Path`, `PathChain`, `BezierLine`/`BezierCurve`, geometry/math types). Bumping this version is one of the most common changes in this repo's history — check the PedroPathing release notes for breaking API changes before bumping.
- `com.pedropathing:telemetry`, `com.bylazar:fullpanels` — the Panels dashboard/telemetry stack used throughout `Tuning.java` (`PanelsTelemetry`, `PanelsField`, `PanelsConfigurables`, `@Configurable`/`@IgnoreConfigurable` annotations).
- Standard FTC SDK modules (`RobotCore`, `Hardware`, `FtcCommon`, `Vision`, etc.) are pinned to a specific SDK release version — keep them in sync with each other when bumping.

`build.gradle` (root), `build.common.gradle`, and `settings.gradle` are vendored FTC SDK build plumbing shared across all FTC teams' forks — avoid editing these except to sync with upstream SDK updates; team-specific build customization belongs in `TeamCode/build.gradle`.

### FtcRobotController module

Contains the stock FTC sample OpModes under `external/samples`, organized by prefix (`Basic`/`Sensor`/`Robot`/`Concept`) as described in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md`. The normal workflow for pulling in a sample is to copy it into the `teamcode` package, not to edit it in place — treat this module as reference material, not a place to add team logic.

## Notes

- `.github/workflows/` contains upstream FIRST-Tech-Challenge fork-maintenance automation (auto-closing/reopening "mistake PRs" opened against forks) — this is infrastructure for the upstream org's fork network, not project CI; it's not something to invoke or debug as part of feature work here.
- Commit history shows PedroPathing dependency bumps are frequent, low-risk, single-line changes to `build.dependencies.gradle` (`com.pedropathing:ftc` version string) — distinct from feature work on `Tuning.java`/`Constants.java`.
