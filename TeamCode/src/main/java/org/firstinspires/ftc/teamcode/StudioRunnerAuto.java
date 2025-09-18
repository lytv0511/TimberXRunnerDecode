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

        // Prebuild trajectories from the *same* starting pose
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
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

        // Run trajectory
        Actions.runBlocking(new SequentialAction(trajectoryActionChosen));

        studioAprilTag.shutdown();
    }

    private int detectTagID() {
        List<AprilTagDetection> detections = studioAprilTag.getDetections();
        if (!detections.isEmpty()) return detections.get(0).id;
        return -1;
    }
}
