# BIOBUZZ Vision — two devices

Two independent vision devices, each with one job:

| Device | Config name | Port | Job | Code |
| --- | --- | --- | --- | --- |
| Limelight 3A | `limelight` | USB 3.0 | **AprilTags only** | `vision/LimelightVision.java`, `StudioAprilTag.java` |
| Logitech webcam | `webcam` | USB 2.0 | **Pollen / nectar color**: yellow, red, blue | `vision/PollenVision.java` |

Odometry is a third, independent system: **2 odometry pods + goBILDA Pinpoint**
(`pinpoint`). No vision result is written into `PinpointLocalizer` — AprilTag
relocalization is **deferred**.

## Webcam color detection (`vision/PollenVision.java`)

One `VisionPortal` on `WebcamName "webcam"` hosts up to three
`ColorBlobLocatorProcessor`s, one per SDK preset color:

- `ENABLE_YELLOW` → `ColorRange.YELLOW` (pollen)
- `ENABLE_RED` → `ColorRange.RED` (nectar)
- `ENABLE_BLUE` → `ColorRange.BLUE` (nectar)

`poll()` scores every filtered blob across all enabled colors and returns the
best `Detection` (color, bearing, range, aspect ratio, circularity, density,
0–100 score). `poll(int color)` restricts to `YELLOW` (0), `RED` (1) or `BLUE` (2).

Range is pinhole: `range = TARGET_DIAM_IN × FOCAL_PX / pixelDiam`, bearing from
horizontal pixel offset × `CAMERA_FOV_DEG / 2`. Sphere filters reject non-round
blobs (`MIN/MAX_ASPECT_RATIO`, `MIN_CIRCULARITY`, `MIN_DENSITY`) and range gates
reject noise.

Tunables (all `@Config`, Dashboard):

| Field | Purpose | Default |
| --- | --- | --- |
| `RESOLUTION_WIDTH` / `RESOLUTION_HEIGHT` | camera resolution | 640 / 480 |
| `TARGET_DIAM_IN` / `FOCAL_PX` | range model | 3.0 / 560 (calibrate) |
| `CAMERA_FOV_DEG` | bearing model | 60 |
| `MIN_AREA_PX` / `MAX_AREA_PX` | blob area gate | 400 / 20000 |
| `MIN_ASPECT_RATIO` / `MAX_ASPECT_RATIO` | sphere check | 1.0 / 1.7 |
| `MIN_CIRCULARITY` / `MIN_DENSITY` | roundness / fill | 0.60 / 0.50 |
| `MIN_RANGE_IN` / `MAX_RANGE_IN` | range gate | 2 / 72 in |
| `BLUR_SIZE` | pre-processing kernel | 5 |
| `ENABLE_YELLOW` / `ENABLE_RED` / `ENABLE_BLUE` | which processors run | all true |

If CPU/FPS is low, drop to 320×240 and re-calibrate `FOCAL_PX`; disable unused
colors.

## Auto-intake assist (`vision/PollenIntakeAssist.java`)

Bound to **held A** in `StudioRunnerTestTeleop`. No odometry — vision closes
the loop directly:

- `bearing → turn` (`KP_BEARING`, `MAX_TURN`), rotate until within `ALIGN_TOL_DEG`
- `range → forward` (`KP_RANGE`, `MAX_FWD`), stop at `STOP_RANGE_IN`
- Energizes `leftIntake`, `rightIntake`, `intakeWheel` while engaged
- `TARGET_COLOR` = -1 (any) or 0/1/2 (yellow/red/blue)
- Timeouts: `TARGET_LOST_MS` bridges short dropouts; `TIMEOUT_S` stops until A is released
- While A is held the assist owns drive + intake; manual RT intake and sticks are suppressed. Flywheel/rubber/Two stay manual.

The webcam is optional at runtime: if `"webcam"` is missing, TeleOp still runs
manually and the assist is disabled.

## Limelight AprilTags (`StudioAprilTag.java`)

`vision/LimelightVision.java` wraps the `Limelight3A` device named `limelight`.
Tag IDs, field map and camera calibration must be configured in the Limelight
web UI — none are supplied by Java. No BioBuzz tag map is loaded yet, so there
is currently no tag-based relocalization.

`StudioAprilTag` exposes `updatePose()`, `getTagCount()`, `getTagId()`,
`getRobotPosition()`, `getRobotOrientation()`, `getYawDegrees()`.
`APRILTAG_PIPELINE` (default 1), `MIN_TAG_AREA_PCT` and `USE_MT2_POSE` are
`@Config`. Results are observations only.

## Calibration (`vision/VisionCalibration.java`)

`VIEW = 0` → webcam: best target, `suggestFocal` at `CALIBRATE_DISTANCE_IN`
(12"), and raw blob stats per color. Toggle `REBUILD` to re-open the portal after
changing tunables.

`VIEW = 1` → Limelight: FPS/status, pipeline, staleness, visible fiducials and
`botpose`. It also reminds you pose still comes from Pinpoint.

## Hardware names

Drive `leftFront`, `rightFront`, `leftBack`, `rightBack`; mechanisms
`leftIntake`, `rightIntake`, `intakeWheel`, `rubber`, `FlyWheel`, `Two`;
odometry `pinpoint`; vision `limelight` and `webcam`.
