# BIOBUZZ Preseason — TimberXRunnerDecode

Base: DECODE (Artifacts) → BIOBUZZ Pollen (3" yellow balls). Kickoff 2026-09-12.

## Current opmodes

Only custom opmodes in TeamCode:

| Opmode | Group | Purpose |
| --- | --- | --- |
| `StudioRunnerTestTeleop` | TeleOp | 2-stick mecanum; RT=intake group, LT=FlyWheel, LB=rubber, RB=Two (reversed), held A=webcam auto-intake assist. |
| `StudioRunnerTestAuto` | Autonomous | 10" forward, 10" right, local 180° spin with 2-pod Pinpoint odometry. |
| `VisionCalibration` | Calibration | Two-device viewer: `VIEW=0` webcam pollen/nectar colors, `VIEW=1` Limelight AprilTags. |

The RoadRunner tuning opmodes in `tuning/` (`TuningOpModes`, `ManualFeedbackTuner`, `SplineTest`, `LocalizationTest`) are kept unchanged.

All other custom opmodes and their helpers were removed: `StudioTestTeleop`, `StudioTestAutoRed/Blue`, `BallDetection`, `VisionChase`, and the entire `autonomous/` package (timed autos, `BioBuzzAutoBase`, `BioBuzzConstants`, `PollenWorkflow`, `autonomous/search/*`).

## Drive & odometry

- `StudioRunnerTestTeleop` uses `BasicMecanumDrive` (open-loop, 4 DcMotors). `rightFront` and `rightBack` are reversed.
- `StudioRunnerTestAuto` uses `MecanumDrive` + `PinpointLocalizer`: 2 odometry pods + goBILDA Pinpoint computer named `pinpoint`.
- The 180° spin in the test auto uses a heading-only loop with zero translation each iteration — it does **not** use RoadRunner `TurnAction`.
- See `ODOMETRY_PINPOINT.md` for pod offsets, directions and calibration.

## Vision & odometry

Two devices, split by job:

- **Limelight 3A `limelight` (USB 3.0)** — AprilTags only. `vision/LimelightVision.java` wraps it; `StudioAprilTag.java` reads fiducials/botpose. No BioBuzz tag map is loaded yet; relocalization is deferred.
- **Logitech webcam `webcam` (USB 2.0)** — pollen/nectar color: yellow, red, blue. `vision/PollenVision.java` runs three SDK color blobs on one portal; `vision/PollenIntakeAssist.java` drives to the best target on held A and runs `leftIntake`, `rightIntake`, `intakeWheel`.

`VisionCalibration` switches between the two devices with `VIEW` (0=webcam, 1=Limelight). See `VISION_BIOBUZZ.md`.

Odometry is independent of vision: **2 odometry pods + goBILDA Pinpoint** (`pinpoint`). Neither device writes into the localizer.

## Hardware names

Drive: `leftFront`, `rightFront`, `leftBack`, `rightBack`.
Mechanisms: `leftIntake`, `rightIntake` (CRServo), `intakeWheel`, `rubber`, `FlyWheel`, `Two`.
Odometry: `pinpoint`. Vision: `limelight` (tags), `webcam` (color).

Old DECODE hardware (launcher/sorter/elevator/color sensor/signal servo) is no longer mapped.
