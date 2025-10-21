package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "AimBotPro", group = "TeleOp")
public class AimBotPro extends OpMode {

    private MecanumDrive drive;
    private StudioAprilTag studioAprilTag;

    private DcMotorEx launcherRight, launcherLeft;
    private Servo rampServo;

    private boolean aimingMode = false;
    private boolean lastXToggle = false;

    // Tuning gains for aiming
    public static double kP_turn = 0.02;   // per degree of yaw error
    public static double kP_strafe = 0.05; // per inch of x offset
    public static double maxPower = 0.25;  // clamp to avoid jerk

    // Launcher constants
    public static final double MAX_RPM = 6000;
    public static final int TICKS_PER_REV = 28;
    public static final double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

    // Target tag ID for red goal
    public static int GOAL_TAG_ID = 24;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");

        // Init launcher motors
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");

        launcherRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherRight.setDirection(DcMotorEx.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // Init ramp servo (optional, if present)
        try {
            rampServo = hardwareMap.get(Servo.class, "rampServo");
        } catch (Exception e) {
            rampServo = null;
        }

        telemetry.addLine("TeleOp Aim + Launcher Initialized. Press start when ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle aiming mode with gamepad1.x
        if (gamepad1.x && !lastXToggle) {
            aimingMode = !aimingMode;
        }
        lastXToggle = gamepad1.x;

        double targetVelocity = 0;
        double rampAngle = 0;
        double distance = 0.0;
        double yawError = 0.0;
        boolean tagVisible = false;

        if (aimingMode) {
            AprilTagDetection tag = studioAprilTag.getBestDetection();
            if (tag != null && tag.id == GOAL_TAG_ID && tag.robotPose != null) {
                tagVisible = true;
                // Get robot position relative to tag (robotPose: x=left/right, y=forward/back, z=up/down)
                double yDist = tag.robotPose.getPosition().y;
                double xDistRaw = tag.robotPose.getPosition().x;
                yawError = tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Clamp lateral offset to max allowed for launcher calculation
                double maxLateralOffset = 12.0; // e.g., max 12 inches lateral considered
                double xDist = Range.clip(xDistRaw, -maxLateralOffset, maxLateralOffset);

                // Compute effective distance using Pythagoras
                double effectiveDistance = Math.sqrt(yDist * yDist + xDist * xDist);

                // Apply minor correction for yaw if within ±15 degrees
                if (Math.abs(yawError) <= 15.0) {
                    double yawRad = Math.toRadians(yawError);
                    effectiveDistance = effectiveDistance / Math.cos(yawRad);
                }

                // Use effectiveDistance for launcher calculations
                distance = effectiveDistance;

                double turnPower = Range.clip(yawError * kP_turn, -maxPower, maxPower);
                double strafePower = Range.clip(xDistRaw * kP_strafe, -maxPower, maxPower);

                drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                        new com.acmerobotics.roadrunner.Vector2d(0, -strafePower),
                        -turnPower
                ));

                // Compute launcher RPM and ramp angle based on effective distance
                if (gamepad1.right_trigger > 0.05) {
                    targetVelocity = computeLauncherRPM(distance);
                } else {
                    targetVelocity = 0;
                }
                rampAngle = computeRampAngle(distance);
                if (rampServo != null) {
                    rampServo.setPosition(rampAngle);
                }

                telemetry.addData("Aiming", "Active");
                telemetry.addData("Distance (in)", distance);
                telemetry.addData("Yaw Error (deg)", yawError);
                telemetry.addData("Launcher RPM", targetVelocity * 60.0 / TICKS_PER_REV);
                telemetry.addData("Ramp Angle", rampAngle);
            } else {
                // No tag detected, stop drive and launcher
                drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                        new com.acmerobotics.roadrunner.Vector2d(0, 0),
                        0
                ));
                targetVelocity = 0;
                if (rampServo != null) {
                    rampServo.setPosition(0.0);
                }
                telemetry.addLine("Aiming: No tag detected");
            }
        } else {
            // Manual driver control
            double driveY = -gamepad1.left_stick_y;
            double driveX = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                    new com.acmerobotics.roadrunner.Vector2d(driveY, driveX),
                    turn
            ));

            telemetry.addData("Mode", "Driver Control");
            double trigger = gamepad1.right_trigger;
            if (trigger > 0.05) {
                // Manual launcher control (max power)
                targetVelocity = trigger * MAX_TICKS_PER_SEC;
            } else {
                targetVelocity = 0;
            }
            if (rampServo != null) {
                rampServo.setPosition(0.0);
            }
        }

        // Stop launcher if no tag is visible in aiming mode
        if (aimingMode && !tagVisible) {
            targetVelocity = 0;
        }
        launcherRight.setVelocity(targetVelocity);
        launcherLeft.setVelocity(targetVelocity);

        telemetry.addData("Launcher Target Vel", targetVelocity);
        telemetry.addData("launcherRight Vel", launcherRight.getVelocity());
        telemetry.addData("launcherLeft Vel", launcherLeft.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        studioAprilTag.shutdown();
        launcherRight.setVelocity(0);
        launcherLeft.setVelocity(0);
        if (rampServo != null) {
            rampServo.setPosition(0.0);
        }
    }

    /**
     * Computes the launcher RPM (in ticks/sec) needed for a given distance (inches).
     * This should be calibrated for your actual launcher and field setup.
     */
    private double computeLauncherRPM(double distanceInches) {
        // Example: 24" → 4000 rpm, 48" → 6000 rpm, linear scaling
        double minRPM = 4000;
        double maxRPM = MAX_RPM;
        double minDist = 24.0;
        double maxDist = 48.0;
        double rpm = minRPM + (maxRPM - minRPM) * Range.clip((distanceInches - minDist) / (maxDist - minDist), 0.0, 1.0);
        // Convert RPM to ticks/sec
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    /**
     * Computes the ramp servo position for a given distance (inches).
     * Adjust min/max position for your launcher servo.
     */
    private double computeRampAngle(double distanceInches) {
        // Example: closer = higher angle, farther = lower angle
        double minDist = 24.0;
        double maxDist = 48.0;
        double minPos = 0.25; // High angle
        double maxPos = 0.75; // Low angle
        double t = Range.clip((distanceInches - minDist) / (maxDist - minDist), 0.0, 1.0);
        return minPos + (maxPos - minPos) * t;
    }
}
