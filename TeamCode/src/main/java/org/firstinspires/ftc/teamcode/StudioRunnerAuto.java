package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "StudioRunnerAuto", group = "Studio")
public class StudioRunnerAuto extends LinearOpMode {

    private StudioAprilTag studioAprilTag;
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        // Start pose
        Pose2d initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);

        // Init AprilTag
        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        // Prebuild trajectories from the *same* starting pose using valid RR 1.0 conventions:
        // tab1: move forward (lineToY), then strafe, then turn, then lineToX
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(36)                          // Move forward to y=36
                .strafeTo(new Vector2d(36, 48))       // Strafe to (36,48)
                .turn(Math.toRadians(90))             // Turn 90 degrees
                .lineToX(44)                          // Move to x=44 facing current heading
                .waitSeconds(3);

        // tab2: move to a diagonal point using splineTo
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(18, 37), Math.toRadians(30)) // Spline to diagonal
                .waitSeconds(3);

        // tab3: strafe to a point and wait
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);

        // Close claw or other init actions if needed
        // Actions.runBlocking(claw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

        // Poll AprilTag once at start
        int tagId = detectTagID();
        telemetry.addData("Detected Tag ID", tagId);
        telemetry.update();

        Action trajectoryActionChosen;
        if (tagId == 21) {
            trajectoryActionChosen = tab1.build();
        } else if (tagId == 22) {
            trajectoryActionChosen = tab2.build();
        } else if (tagId == 23) {
            trajectoryActionChosen = tab3.build();
        } else {
            // fallback simple move
            trajectoryActionChosen = drive.actionBuilder(initialPose)
                    .lineToY(-12)
                    .build();
        }

        // Real-time pose telemetry during trajectory execution
        final boolean[] running = {true};
        Thread telemetryThread = new Thread(() -> {
            while (running[0] && !isStopRequested()) {
                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("X (in)", pose.position.x);
                telemetry.addData("Y (in)", pose.position.y);
                telemetry.addData("Heading (deg)", pose.heading);
                telemetry.update();
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        telemetryThread.start();

        // Run trajectory
        Actions.runBlocking(new SequentialAction(trajectoryActionChosen));

        // Stop telemetry after trajectory finishes
        running[0] = false;
        try {
            telemetryThread.join();
        } catch (InterruptedException ignored) {}

        studioAprilTag.shutdown();
    }

    private int detectTagID() {
        List<AprilTagDetection> detections = studioAprilTag.getDetections();
        if (!detections.isEmpty()) return detections.get(0).id;
        return -1;
    }
    /**
     * Converts a raw encoder rotational value from radians to degrees.
     * @param rotationRadians the rotation in radians
     * @return the rotation in degrees
     */
    private double encoderRadiansToDegrees(double rotationRadians) {
        return Math.toDegrees(rotationRadians);
    }
}
