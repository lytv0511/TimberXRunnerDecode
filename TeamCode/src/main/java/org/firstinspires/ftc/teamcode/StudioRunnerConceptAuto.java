package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
@Autonomous(name = "StudioRunnerConceptAuto", group = "Autonomous")
public class StudioRunnerConceptAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing AprilTag...");
        telemetry.update();

        StudioAprilTag aprilTag = new StudioAprilTag();
        aprilTag.init(hardwareMap, "Webcam 1");

        telemetry.addLine("AprilTag Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            aprilTag.shutdown();
            return;
        }

        while (opModeIsActive()) {
            aprilTag.updatePose();

            int tagId = aprilTag.getTagId();
            String tagName = aprilTag.getTagName();
            double yaw = aprilTag.getYawDegrees();
            org.firstinspires.ftc.robotcore.external.navigation.Position pos = aprilTag.getRobotPosition();

            telemetry.addLine("=== AprilTag Detection ===");
            if (tagId != -1) {
                telemetry.addData("Tag ID", tagId);
                telemetry.addData("Tag Name", tagName != null ? tagName : "Unknown");
                if (pos != null) {
                    telemetry.addData("X (in)", "%.2f", pos.x);
                    telemetry.addData("Y (in)", "%.2f", pos.y);
                    telemetry.addData("Z (in)", "%.2f", pos.z);
                } else {
                    telemetry.addLine("No valid position detected.");
                }
                telemetry.addData("Yaw (deg)", "%.2f", yaw);
            } else {
                telemetry.addLine("No tag detected.");
            }
            telemetry.update();
        }

        aprilTag.shutdown();
    }
}