package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "AimBotPro", group = "TeleOp")
public class AimBotPro extends OpMode {

    private MecanumDrive drive;
    private StudioAprilTag studioAprilTag;

    private DcMotorEx launcher1, launcher2;

    private boolean aimingMode = false;
    private boolean lastAToggle = false;

    // Tuning gains
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
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        launcher1.setDirection(DcMotorEx.Direction.FORWARD);
        launcher2.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addLine("TeleOp Aim + Launcher Initialized. Press start when ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle aiming mode with gamepad1.x
        if (gamepad1.x && !lastAToggle) {
            aimingMode = !aimingMode;
        }
        lastAToggle = gamepad1.x;

        double targetVelocity = 0;

        if (aimingMode) {
            AprilTagDetection tag = studioAprilTag.getBestDetection();

            if (tag != null && tag.id == GOAL_TAG_ID && tag.robotPose != null) {
                double yawError = tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                double xError = tag.robotPose.getPosition().x;
                double distance = tag.robotPose.getPosition().y;

                double turnPower = Range.clip(yawError * kP_turn, -maxPower, maxPower);
                double strafePower = Range.clip(xError * kP_strafe, -maxPower, maxPower);

                drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                        new com.acmerobotics.roadrunner.Vector2d(0, -strafePower),
                        -turnPower
                ));

                telemetry.addData("Aiming", "Active");
                telemetry.addData("Yaw Error (deg)", yawError);
                telemetry.addData("X Error (in)", xError);
                telemetry.addData("Distance (in)", distance);

                // Distance dependent velocity
                if (gamepad1.right_trigger > 0.05) {
                    targetVelocity = getVelocityForDistance(distance);
                }

            } else {
                // Stop drive with no AprilTag
                drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                        new com.acmerobotics.roadrunner.Vector2d(0, 0),
                        0
                ));
                telemetry.addLine("Aiming: No tag detected");
            }
        } else {
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
                targetVelocity = trigger * MAX_TICKS_PER_SEC;
            }
        }

        launcher1.setVelocity(targetVelocity);
        launcher2.setVelocity(targetVelocity);

        telemetry.addData("Launcher Target Vel", targetVelocity);
        telemetry.addData("Launcher1 Vel", launcher1.getVelocity());
        telemetry.addData("Launcher2 Vel", launcher2.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        studioAprilTag.shutdown();
        launcher1.setVelocity(0);
        launcher2.setVelocity(0);
    }

    // Experimental calibration needed
    private double getVelocityForDistance(double distanceInches) {
        // Very rough linear velocity scale with respect to distance
        // 24" → 4000 rpm, 48" → 6000 rpm
        double minVel = (4000 / 60.0) * TICKS_PER_REV;
        double maxVel = MAX_TICKS_PER_SEC;
        double scaled = Range.clip((distanceInches / 48.0), 0.0, 1.0);
        return minVel + (maxVel - minVel) * scaled;
    }
}
