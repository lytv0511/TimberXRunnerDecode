package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@Autonomous(name="StudioRunnerAuto", group="Studio")
public class StudioRunnerAuto extends LinearOpMode {

    private StudioAprilTag studioAprilTag;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize RR drive with odometry
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, startPose);

        // Initialize AprilTag camera
        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        // Step 1: move forward ~24 inches to get in range for detection
        Pose2d prepPose = drive.getPoseEstimate();
        TrajectoryActionBuilder prepBuilder = drive.actionBuilder(prepPose)
                .lineTo(prepPose.position.x + 24 * Math.cos(prepPose.heading.toDouble()),
                        prepPose.position.y + 24 * Math.sin(prepPose.heading.toDouble()));
        Actions.runBlocking(prepBuilder.build());

        // Step 2: poll AprilTag until detected
        int tagId = -1;
        while (opModeIsActive() && tagId == -1) {
            tagId = detectTagID();
            telemetry.addData("Tag ID", tagId);
            telemetry.update();
            sleep(50);
        }
        telemetry.addData("Detected Tag ID", tagId);
        telemetry.update();

        // Step 3: build and execute tag-specific trajectory sequence
        Pose2d currentPose = drive.getPoseEstimate();
        TrajectoryActionBuilder ab = drive.actionBuilder(currentPose);
        Action chosenSequence;

        switch (tagId) {
            case 21: // GPP
                chosenSequence = ab
                        .lineTo(currentPose.position.x + 36 * Math.cos(currentPose.heading.toDouble()),
                                currentPose.position.y + 36 * Math.sin(currentPose.heading.toDouble()))
                        .strafeTo(new Vector2d(currentPose.position.x + 36, currentPose.position.y + 24))
                        .turn(Math.toRadians(90))
                        .build();
                break;
            case 22: // PGP
                chosenSequence = ab
                        .lineTo(currentPose.position.x - 36 * Math.cos(currentPose.heading.toDouble()),
                                currentPose.position.y - 36 * Math.sin(currentPose.heading.toDouble()))
                        .strafeTo(new Vector2d(currentPose.position.x - 36, currentPose.position.y + 24))
                        .turn(Math.toRadians(-90))
                        .build();
                break;
            case 23: // PPG
                chosenSequence = ab
                        .strafeTo(new Vector2d(currentPose.position.x + 36, currentPose.position.y))
                        .lineTo(currentPose.position.x + 36 * Math.cos(currentPose.heading.toDouble()),
                                currentPose.position.y + 24 * Math.sin(currentPose.heading.toDouble()))
                        .turn(Math.toRadians(90))
                        .build();
                break;
            default: // fallback
                telemetry.addLine("No valid tag detected, running fallback");
                telemetry.update();
                chosenSequence = ab.lineTo(currentPose.position.x - 12, currentPose.position.y).build();
                break;
        }

        // Step 4: run the chosen trajectory sequence
        Actions.runBlocking(chosenSequence);

        // Shutdown camera
        studioAprilTag.shutdown();
    }

    private int detectTagID() {
        List<AprilTagDetection> detections = studioAprilTag.getDetections();
        if (!detections.isEmpty()) return detections.get(0).id;
        return -1;
    }
}
