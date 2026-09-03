# BIOBUZZ Preseason Hygiene — TimberXRunnerDecode

Date: 2026-08-24 | Base: DECODE (Artifacts) → BIOBUZZ Pollen (3" balls) | Kickoff 2026-09-12

## What was done (hardware untouched)

**Build:**
- `TeamCode/build.gradle:39` removed duplicate `core:1.0.1`.

**Code hygiene:**
- `StudioTestTeleop.java` deleted two giant commented `defaultLaunchSequence` variants (~250 lines).
- `StudioTestAutoRed.java:219` deleted commented launch clone (140 lines).
- Autos stripped of `gamepad1.x/dpad` branches (auto has no driver) — pure `ElapsedTime` + `idle()` loops.

**Autonomous planning framework (new, field-agnostic):**
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/
  BioBuzzConstants.java      // Pollen dia 3", intake powers, nudge distances, @Config tunable
  BioBuzzAutoBase.java       // LinearOpMode base: init drive+4 motors, pose helpers, alliance mirroring
  PollenWorkflow.java        // Preview tasks: floor single, line(3), pile(3), offWall, fromCorner, navigate+intake
  PreseasonBioBuzzBench.java // Skill Builder bench MODE 0-5 selectable in Dashboard
```
- `StudioTestAutoRed/Blue.java` now extend `BioBuzzAutoBase` — 80 LOC each, share `launchSequence()` + `PollenWorkflow`. Red sign `+1`, Blue `-1` mirrors `turnBy()`/`strafeBy()`. Trajectories are **relative** (`driveLineToX(delta)`, `turnBy(deg)`) until field CAD lands. Replace `launchSequence()` with BIOBUZZ outtake post-kickoff.

## How to use (preseason)

1. Buy Pollen pack (AndyMark) — same handling as Artifacts.
2. Tune `BioBuzzConstants` in Dashboard: `NUDGE_SHORT/LONG`, `WALL_APPROACH`, `CORNER_STRAFE`, `PILE_SPACING`.
3. Run `PreseasonBioBuzzBench` MODE 0→5 on tiles:
   - 0 floor, 1 line, 2 pile, 3 wall, 4 corner, 5 spline waypoints. Iterate intake geometry for wall/corner.
4. Red/Blue autos are drill-ready: `drive → launch → wall/pile intake → return → launch`.

## Next after kickoff

- Replace placeholder distances (`-51in`, `10°`, etc.) with field coordinates; switch `lineToX` → `splineTo/strafeTo(Vector2d)`.
- Decide sorter: BIOBUZZ preview shows bulk transfer, not GPP/PGP sorting — likely delete `sorterTo()` / `augPos*` and make intake hold N pollen.
- Tune localizer: `MecanumDrive.java:253` uses `ThreeDeadWheelLocalizer` on drive encoders — calibrate or swap to `PinpointLocalizer` (offsets still 0).
- Pollen vision: retune `BallDetection.java`/`StudioAprilTag.java` ColorRange from green/purple to Pollen yellow.

Hardware names preserved: `leftFront/leftBack/rightBack/rightFront`, `imu`, `launcherFlywheel`, `launcherElevator`, `sorter`, `intake`, `Webcam 1`, `colorSensor`, `signalServo`.
