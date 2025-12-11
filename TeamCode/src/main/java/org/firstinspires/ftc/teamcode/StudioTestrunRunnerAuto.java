package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Direct camera-relative AprilTag distance & angle tracker.
 * Uses det.ftcPose.x, y, z without robotPose or field mapping.
 */
@Autonomous(name = "StudioTestrunRunnerAuto", group = "Autonomous")
public class StudioTestrunRunnerAuto extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing AprilTag...");
        telemetry.update();

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("AprilTag Initialized — Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.isEmpty()) {
                telemetry.addLine("No AprilTag detected.");
                telemetry.update();
                continue;
            }

            AprilTagDetection det = detections.get(0);

            double x = det.ftcPose.x;  // forward distance (in)
            double y = det.ftcPose.y;  // left-right (in)
            double z = det.ftcPose.z;  // up-down (ignored for flat distance)

            double flatDistance = Math.hypot(x, y) * 1.1;
            double theta = Math.toDegrees(Math.atan2(y, x));

            telemetry.addLine("=== Direct Camera → Tag ===");
            telemetry.addData("Forward x", "%.2f in", x);
            telemetry.addData("Left y", "%.2f in", y);
            telemetry.addData("Vertical z", "%.2f in", z);

            telemetry.addData("Flat Distance", "%.2f in", flatDistance);
            telemetry.addData("Horizontal Angle θ", "%.2f°", theta);

            telemetry.addData("Tag Yaw", "%.2f°", det.ftcPose.yaw);
            telemetry.addData("Tag Pitch", "%.2f°", det.ftcPose.pitch);
            telemetry.addData("Tag Roll", "%.2f°", det.ftcPose.roll);

            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}