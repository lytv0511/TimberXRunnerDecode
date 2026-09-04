package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.PollenVision;

/**
 * "Chase the ball" demo — no intake, only the 4 drive motors.
 * Looks for a green or purple wiffle ball via the camera and drives toward it.
 *
 * Color selection (Dashboard):
 *   PollenVision.COLOR_MODE = 4 (ARTIFACT_GREEN) for green, 5 (ARTIFACT_PURPLE) for purple.
 *   Toggle VisionChase.REBUILD (0->1->0) to apply a color change live.
 *
 * Drive behavior:
 *   - Turns toward the ball (bearing PD) and drives forward.
 *   - Stops when the ball is close (STOP_RANGE_IN) or out of view.
 *   - Gamepad triggers override: drive manually with left/right sticks if held
 *     (useful to reposition when no ball in view).
 *
 * Only uses: leftFront, leftBack, rightBack, rightFront drive motors + webcam.
 */
@Config
@TeleOp(name = "VisionChase", group = "Vision")
public class VisionChase extends LinearOpMode {

    // ---- Drive/tuning ----
    public static double STOP_RANGE_IN = 8.0;      // stop when this close to ball
    public static double BASE_FWD_POWER = 0.35;    // forward speed toward ball
    public static double TURN_KP = 0.030;          // power per degree of bearing error
    public static double MAX_TURN_POWER = 0.55;
    public static double LOST_TIMEOUT_S = 0.8;     // stop after no detection this long

    // ---- Hot-reload toggle (0->1->0 in Dashboard to apply color change) ----
    public static double REBUILD = 0;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        PollenVision vision = new PollenVision();
        vision.init(hardwareMap, "Webcam 1");

        telemetry.addLine("VisionChase ready — COLOR_MODE 4=green 5=purple");
        telemetry.addLine("Point camera at a ball. Gamepad triggers for manual override.");
        telemetry.update();

        waitForStart();

        double lastRebuild = REBUILD;
        // Track last time we saw a ball for the lost-timeout
        long lastSeenMs = System.currentTimeMillis();
        boolean seen = false;

        while (opModeIsActive()) {
            // Hot-reload color change
            if (REBUILD != lastRebuild) {
                lastRebuild = REBUILD;
                vision.rebuild(hardwareMap, "Webcam 1", new android.util.Size(640, 480));
            }

            PollenVision.Detection d = vision.poll();

            // Manual override: if any gamepad stick active, let driver take over
            boolean manual = gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0
                    || gamepad1.right_stick_x != 0 || gamepad2.left_stick_y != 0
                    || gamepad2.left_stick_x != 0 || gamepad2.right_stick_x != 0;

            if (manual) {
                driveManual(drive);
                seen = false;
                lastSeenMs = System.currentTimeMillis();
            } else if (d != null) {
                lastSeenMs = System.currentTimeMillis();
                seen = true;

                double bearing = d.bearingDeg;
                double range = d.rangeIn;

                telemetry.addData("target", "range %.1f in  bearing %.1f deg", range, bearing);

                if (range <= STOP_RANGE_IN) {
                    stopDrive(drive);
                    telemetry.addLine("REACHED");
                } else {
                    // Turn toward ball + drive forward
                    double turn = clamp(TURN_KP * bearing, -MAX_TURN_POWER, MAX_TURN_POWER);
                    // Slow down as we approach to avoid overshoot
                    double fwdMult = clamp((range - STOP_RANGE_IN) / 12.0, 0.15, 1.0);
                    double fwd = BASE_FWD_POWER * fwdMult;
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, 0), turn));
                }
            } else {
                // Lost ball — stop after timeout
                if (System.currentTimeMillis() - lastSeenMs > LOST_TIMEOUT_S * 1000) {
                    stopDrive(drive);
                    telemetry.addLine("no target");
                } else {
                    // brief coast to avoid jitter
                }
                seen = false;
            }

            telemetry.addData("color", PollenVision.colorModeName());
            telemetry.addData("raw blobs", vision.getRawBlobs().size());
            telemetry.addData("state", seen ? "SEEKING" : (manual ? "MANUAL" : "IDLE"));
            telemetry.update();
        }

        stopDrive(drive);
        vision.close();
    }

    private void driveManual(MecanumDrive drive) {
        // Mix gamepads: left stick for strafe+forward, right stick for turn
        double forward = -gamepad1.left_stick_y + -gamepad2.left_stick_y;
        double strafe  =  gamepad1.left_stick_x +  gamepad2.left_stick_x;
        double turn    =  gamepad1.right_stick_x + gamepad2.right_stick_x;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, strafe), turn));
    }

    private void stopDrive(MecanumDrive drive) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
