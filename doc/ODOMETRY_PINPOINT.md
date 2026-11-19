# Odometry — 2 Pods + goBILDA Pinpoint

`MecanumDrive` now uses `PinpointLocalizer` instead of `ThreeDeadWheelLocalizer`.

## Hardware

- One **goBILDA Pinpoint Odometry Computer** in the robot config named exactly `pinpoint` (I2C).
- Two dead-wheel odometry pods plugged into the Pinpoint:
  - **X / forward (parallel) pod** — counts up when the robot moves forward.
  - **Y / strafe (perpendicular) pod** — counts up when the robot moves left.
- The Pinpoint's internal IMU supplies heading. A separate `imu` device is **not** used for odometry (`MecanumDrive` still constructs a lazy IMU object, but nothing calls it in the odometry path).

Drive motors are still `leftFront`, `rightFront`, `leftBack`, `rightBack`.

## Dashboard tuning — `PinpointLocalizer.PARAMS`

| Field | Meaning |
| --- | --- |
| `use4BarPods` | `true` uses the built-in goBILDA 4-bar resolution (2000 CPR / 32 mm wheel = 19.894 ticks/mm). |
| `customTicksPerMm` | Used when `use4BarPods = false` (goBILDA swingarm = 13.26291192). |
| `parYTicks` | Lateral offset of the X (forward) pod from the tracking point, in pod ticks. Left = positive. |
| `perpXTicks` | Longitudinal offset of the Y (strafe) pod from the tracking point, in pod ticks. Forward = positive. |
| `reversePar` / `reversePerp` | Flip if a pod counts the wrong way (X must increase moving forward, Y moving left). |

Offsets are converted from pod ticks to mm before `setOffsets()` is called. Start with both offsets at `0` and measure on the robot.

## Test opmode — `StudioRunnerTestAuto`

`Autonomous` group, name `StudioRunnerTestAuto`. With Dashboard tunables `DISTANCE_IN = 10` and `TURN_DEG = 180` it runs:

1. `lineToX(start.x + 10)` — forward 10 in
2. `strafeTo(start.x + 10, start.y - 10)` — right 10 in (RoadRunner +Y is left)
3. **Local, in-place 180° spin** at the current pose.

The spin intentionally does **not** use RoadRunner's `.turn(...)`/`TurnAction`. That action runs the holonomic pose controller, which corrects position error as well as heading — if the pose estimate drifted during the strafe, the turn can translate the robot (it looked like it drove back toward the origin). Instead, the test runs its own heading-only P-loop that commands `new PoseVelocity2d(new Vector2d(0, 0), power)` every iteration, so translational velocity is exactly zero and the robot can only rotate about its own center. Heading feedback comes from the Pinpoint localizer.

Tunables for the spin: `TURN_KP`, `MIN_TURN_POWER` (friction floor), `MAX_TURN_POWER`, `TURN_TOL_DEG`, `TURN_TIMEOUT_S`. If the robot drifts during the spin, re-check Pinpoint heading sign and pod directions (`reversePar` / `reversePerp`); if it under-rotates, raise `MIN_TURN_POWER` or `TURN_KP`.

It prints start / before-turn / end pose to telemetry so you can verify odometry accuracy. Run it on blocks with the robot stationary during Pinpoint init, and recalibrate the Pinpoint IMU before each run.
