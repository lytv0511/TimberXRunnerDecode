package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Odometry test auto: forward 10in, right 10in, then a LOCAL in-place 180deg spin.
 *
 * The spin deliberately does NOT use RoadRunner's turn()/TurnAction. That action
 * runs the holonomic controller against the target pose, which also corrects
 * position error and can translate the robot during the turn. Here the translational
 * component of every drive command is exactly (0,0), so the robot can only rotate
 * about its own center. Heading feedback comes from the 2-pod Pinpoint localizer.
 *
 * RoadRunner frame: +X forward from start, +Y left, so "right" means decreasing Y.
 */
@Config
@Autonomous(name = "StudioRunnerTestAuto", group = "Autonomous")
public class StudioRunnerTestAuto extends LinearOpMode {

    public static double DISTANCE_IN = 10.0;
    public static double TURN_DEG = 180.0;

    // Heading-only controller
    public static double TURN_KP = 1.5;
    public static double MIN_TURN_POWER = 0.18; // overcome static friction near the target
    public static double MAX_TURN_POWER = 0.5;
    public static double TURN_TOL_DEG = 2.0;
    public static double TURN_TIMEOUT_S = 3.0;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("StudioRunnerTestAuto ready (Pinpoint 2-pod odometry)");
        telemetry.addData("Forward / right", "%.1f in / %.1f in", DISTANCE_IN, DISTANCE_IN);
        telemetry.addData("Local turn", "%.1f deg", TURN_DEG);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Pose2d start = drive.localizer.getPose();
        double forwardX = start.position.x + DISTANCE_IN;
        double rightY = start.position.y - DISTANCE_IN;

        // 1) Forward then right — closed-loop trajectory following.
        Actions.runBlocking(
                drive.actionBuilder(start)
                        .lineToX(forwardX)
                        .strafeTo(new Vector2d(forwardX, rightY))
                        .build()
        );

        // 2) Pure local rotation: translation is hard-zeroed every iteration.
        drive.updatePoseEstimate();
        double heading0 = drive.localizer.getPose().heading.toDouble();
        double targetDelta = Math.toRadians(TURN_DEG);

        ElapsedTime turnTimer = new ElapsedTime();
        while (opModeIsActive() && turnTimer.seconds() < TURN_TIMEOUT_S) {
            drive.updatePoseEstimate();
            double turned = drive.localizer.getPose().heading.toDouble() - heading0;
            double error = wrapPi(targetDelta - turned);

            if (Math.abs(error) <= Math.toRadians(TURN_TOL_DEG)) break;

            double power = Math.max(-MAX_TURN_POWER,
                    Math.min(MAX_TURN_POWER, TURN_KP * error));
            if (Math.abs(power) < MIN_TURN_POWER) {
                power = Math.copySign(MIN_TURN_POWER, error);
            }

            // (0,0) translation => rotation about the robot's own center only.
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), power));
            idle();
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        // 3) Report final pose.
        drive.updatePoseEstimate();
        Pose2d end = drive.localizer.getPose();

        telemetry.addLine("Done");
        telemetry.addData("Start", "%.2f, %.2f, %.1f deg", start.position.x, start.position.y,
                Math.toDegrees(start.heading.toDouble()));
        telemetry.addData("Before turn", "%.2f, %.2f, %.1f deg", forwardX, rightY,
                Math.toDegrees(heading0));
        telemetry.addData("End", "%.2f, %.2f, %.1f deg", end.position.x, end.position.y,
                Math.toDegrees(end.heading.toDouble()));
        telemetry.update();

        sleep(1000);
    }

    /** Wrap an angle to [-pi, pi]. */
    private static double wrapPi(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
