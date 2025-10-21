package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name="StudioRunnerAuto", group = "Autonomous")
public class StudioRunnerAuto extends LinearOpMode {
    private StudioAprilTag studioAprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");
        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        int tagId;
        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = studioAprilTag.getDetections();
            studioAprilTag.updatePose();

            if (!detections.isEmpty()) {
                tagId = detections.get(0).id;
                telemetry.addData("Detected Tag ID", tagId);

                Position pos = studioAprilTag.getRobotPosition();
                YawPitchRollAngles ori = studioAprilTag.getRobotOrientation();

                if (pos != null && ori != null) {
                    telemetry.addData("Robot X", "%.2f", pos.x);
                    telemetry.addData("Robot Y", "%.2f", pos.y);
                    telemetry.addData("Robot Z", "%.2f", pos.z);
                    telemetry.addData("Robot Yaw", "%.2f", ori.getYaw(AngleUnit.DEGREES));
                    telemetry.addData("Robot Pitch", "%.2f", ori.getPitch(AngleUnit.DEGREES));
                    telemetry.addData("Robot Roll", "%.2f", ori.getRoll(AngleUnit.DEGREES));
                }

            } else {
                telemetry.addLine("No tag detected");
            }
            telemetry.update();
            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(20)
                        .build());

        sleep(500);

        tagId = detectTagID();
        telemetry.addData("Detected Tag ID after scanning", tagId);

        String tagName = studioAprilTag.getTagName();
        if (tagName != null) {
            telemetry.addData("Tag Name", tagName);
        }

        Position pos = studioAprilTag.getRobotPosition();
        YawPitchRollAngles ori = studioAprilTag.getRobotOrientation();
        if (pos != null && ori != null) {
            telemetry.addData("Robot X", "%.2f", pos.x);
            telemetry.addData("Robot Y", "%.2f", pos.y);
            telemetry.addData("Robot Z", "%.2f", pos.z);
            telemetry.addData("Robot Yaw", "%.2f", ori.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Robot Pitch", "%.2f", ori.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Robot Roll", "%.2f", ori.getRoll(AngleUnit.DEGREES));
        }
        telemetry.update();

        Action trajectoryActionChosen;
        Pose2d currentPose = drive.localizer.getPose();
        if (tagId == 21) {
            trajectoryActionChosen = drive.actionBuilder(currentPose)
                    .splineTo(new Vector2d(71, 0), Math.toRadians(-90))
                    .lineToY(-48)
                    .waitSeconds(0.5)
                    .splineTo(new Vector2d(95 + 8, -40 + 8), Math.toRadians(-62))
                    .build();
        } else if (tagId == 22) {
            trajectoryActionChosen = drive.actionBuilder(currentPose)
                    .lineToX(36)
                    .strafeTo(new Vector2d(36, 48))
                    .build();
        } else if (tagId == 23) {
            trajectoryActionChosen = drive.actionBuilder(currentPose)
                    .strafeTo(new Vector2d(46, 30))
                    .build();
        } else {
            trajectoryActionChosen = drive.actionBuilder(currentPose)
                    .waitSeconds(0.1)
                    .build();
        }

        Actions.runBlocking(
                drive.actionBuilder(currentPose)
                        .lineToX(103)
                        .turn(Math.toRadians(-90))
                        .lineToY(-32)
                        .turn(Math.toRadians(28))
                        .build()
        );
        sleep(5000);
        Actions.runBlocking(trajectoryActionChosen);

        studioAprilTag.shutdown();
    }

    private int detectTagID() {
        List<AprilTagDetection> detections = studioAprilTag.getDetections();
        if (!detections.isEmpty()) return detections.get(0).id;
        return -1;
    }
}
