# Real Autonomous — Vision Search, Not Hard-Coded Paths

**Problem you hit:** hard-coded `lineToX(-51)` / `turn(10)` / `strafe(-10)` dies when pollen isn’t there, and `Actions.runBlocking` blocks vision so you never see it. This replaces paths with **probability + servo**.

## Architecture (new files)

```
vision/PollenVision.java                 // one Portal + one blob, not 3-on-1
autonomous/search/SearchCell.java        // name, Pose2d, prior 0..1
autonomous/search/SearchPlanner.java     // picks max utility = prior*decay - dist*cost
autonomous/search/ScanAction.java        // sweep ±35° at cell, poll every 10ms
autonomous/search/VisionApproachAction.java // PD servo: fwd = Kp*rangeErr, turn = Kp*bearing
autonomous/search/SearchAndIntakeAction.java // loop: pick → drive → scan → servo → intake
autonomous/BioBuzzAutoBase.java          // now owns vision + planner, runSearchAndIntake()
autonomous/BioBuzzSearchAuto.java        // production auto: 3" backoff → search 5 in 26s
```

**Key fixes vs BallDetection/StudioAprilTag:**
- Single `ColorBlobLocatorProcessor` on `VisionPortal` at `640x480` (was 3 processors at 320x240 → starvation). `PollenVision.poll()` copies list before `filterByCriteria` to avoid mutating original.
- Pinhole `range = (3" * FOCAL_PX) / pixD` with `FOCAL_PX` calibrated, not `(w/2)*(5/3)-2`.
- Bearing via `FOV` linear model, not hard pixels `110/200`.
- Servo is an `Action` that calls `drive.setDrivePowers` every loop, so vision never blocked.

## How search works

1. **Priors** in `SearchPlanner` reflect BIOBUZZ preview: *"pollen rolls against border, into corners"* → `wall 0.40`, `corner 0.35`, `line/pile 0.30`, `center 0.20`. Replace with real field spawns on 2026-09-12 via `planner.setCells(real)`.
2. `next()` scores `utility - dist*0.008`. Failures decay `0.35^failures`. Success boosts neighbors `1.4` within 18" (piles).
3. At cell, `ScanAction` sweeps while polling. If blob `range<12"` + `|bearing|<5°`, hold `0.22s` then count as intake → boosts nearby cells.
4. Opportunistic: while **driving** to cell, if blob appears `<30"` + `|bearing|<22°`, abort trajectory and servo immediately (saves time).

## Tuning (Dashboard — all `@Config`)

1. **Camera:** `PollenVision.FOCAL_PX 420`, `CAMERA_FOV_DEG 60`, `MIN_AREA 350`, `BLUR 5`. Calibrate: put 3" pollen at 12", adjust `FOCAL_PX` until `getRange()==12` (`suggestFocal()` helper). Switch `POLLEN_COLOR` from `ARTIFACT_GREEN` to real yellow HSV after you see DS preview.

2. **Servo:** `VisionApproachAction.Kp_bearing 0.035`, `Kp_range 0.055`, `TARGET_RANGE 5.0"`, `LOST_TIMEOUT 0.6s`. If oscillates, lower Kp_bearing; if stalls far, raise Kp_range. `MAX_FWD 0.55` keeps it controllable near walls.

3. **Planner:** `SearchPlanner.DISTANCE_COST_PER_IN 0.008`, `NEIGHBOR_BOOST 1.4`. After practice logs, raise priors for where you actually find pollen.

4. **Bench test:** `PreseasonBioBuzzBench` MODE 6 runs pure vision search (no hard path). MODE 3/4 test wall/corner priors isolated. Set `BioBuzzSearchAuto.TARGET_POLLEN 5`, `TIMEOUT 26s`, `ALLIANCE_IS_BLUE false/true`.

## Migration

- Old `StudioTestAutoRed/Blue` keep hard-coded for regression; set `USE_VISION_SEARCH=true` in Dashboard to run vision on same opmode.
- New autos: use `BioBuzzSearchAuto` (or copy it) — only hard-coded line is `SAFE_BACKOFF 3.5"` to clear wall, everything else is search.
- Post-kickoff: `planner.setCells(List.of(at("spawnA", x, y, hdg, 0.5)))` from field CAD, change scoring from `intake hold` to base-station deposit spline.

Hardware untouched: `leftFront/.../imu/Webcam 1/intake/launcherFlywheel/launcherElevator/sorter/colorSensor`.
