package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * Vision calibration tool for BIOBUZZ Pollen detection.
 * Run this TeleOp on the robot with Pollen balls at known distances.
 *
 * What to do:
 * 1. Place Pollen ball at 12" from camera
 * 2. Look at Driver Station telemetry — see if contours appear
 * 3. If no detection: change COLOR_MODE (0-6) in Dashboard until contours show
 * 4. If COLOR_MODE=6 (custom): tune CUSTOM_H/S/V until yellow ball is detected
 * 5. Read "pixD" from telemetry — call suggestFocal(pixD, 12.0) → set FOCAL_PX
 * 6. Move ball to 6", 12", 24", 36" — verify "range" matches known distance
 * 7. Tune MIN_AREA_PX to reject floor glare (raise if floor is detected)
 * 8. Tune MIN_SOLIDITY to reject shadows (raise if shadows pass)
 *
 * Dashboard fields (all @Config, live-adjustable):
 *   COLOR_MODE       — 0=YELLOW 1=GREEN 2=RED 3=BLUE 4=ARTIFACT_GREEN 5=ARTIFACT_PURPLE 6=CUSTOM
 *   CUSTOM_H/S/V     — custom HSV bounds (when COLOR_MODE=6)
 *   FOCAL_PX          — pinhole focal length (start at 560 for 640x480)
 *   CAMERA_FOV_DEG    — camera horizontal FOV (check webcam spec)
 *   MIN_AREA_PX       — reject small blobs
 *   MIN_SOLIDITY      — reject unfilled contours
 *   MIN/MAX_ASPECT_RATIO — reject non-spherical blobs
 */
@Config
@TeleOp(name = "VisionCalibration", group = "Calibration")
public class VisionCalibration extends LinearOpMode {

    // Reference distance for focal calibration — place ball at this distance
    public static double CALIBRATE_DISTANCE_IN = 12.0;

    @Override
    public void runOpMode() {
        PollenVision vision = new PollenVision();
        vision.init(hardwareMap, "Webcam 1");

        telemetry.addLine("=== VisionCalibration ===");
        telemetry.addLine("Place Pollen at CALIBRATE_DISTANCE from camera");
        telemetry.addData("Calib dist", CALIBRATE_DISTANCE_IN + " in");
        telemetry.addLine("Tune COLOR_MODE + FOCAL_PX in Dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            PollenVision.Detection d = vision.poll();

            // === Raw blobs (before scoring, for debugging) ===
            List<ColorBlobLocatorProcessor.Blob> raw = vision.getRawBlobs();
            telemetry.addData("Raw blobs", raw.size());
            telemetry.addData("Color mode", PollenVision.COLOR_MODE);
            telemetry.addData("FOCAL_PX", PollenVision.FOCAL_PX);

            if (d != null) {
                RotatedRect box = d.box;
                telemetry.addLine("--- BEST DETECTION ---");
                telemetry.addData("range", "%.1f in", d.rangeIn);
                telemetry.addData("bearing", "%.1f deg", d.bearingDeg);
                telemetry.addData("area", "%.0f px", d.area);
                telemetry.addData("pixD", "%.1f px", d.pixelDiam);
                telemetry.addData("aspectRatio", "%.2f", d.aspectRatio);
                telemetry.addData("solidity", "%.2f", d.solidity);
                telemetry.addData("score", "%.0f", d.score);
                telemetry.addData("cx/cy", "%.0f / %.0f", d.cx, d.cy);

                // Suggest focal if user placed ball at known distance
                double suggested = PollenVision.suggestFocal(d.pixelDiam, CALIBRATE_DISTANCE_IN);
                telemetry.addData("suggestFocal", "%.0f (if ball at %.0f in)",
                        suggested, CALIBRATE_DISTANCE_IN);
            } else {
                telemetry.addLine("--- NO DETECTION ---");
                telemetry.addLine("Adjust COLOR_MODE or CUSTOM HSV in Dashboard");
            }

            // Show all raw blobs for debugging
            int idx = 0;
            for (ColorBlobLocatorProcessor.Blob b : raw) {
                RotatedRect box = b.getBoxFit();
                double pixD = Math.max(box.size.width, box.size.height);
                double range = (PollenVision.POLLEN_DIAM_IN * PollenVision.FOCAL_PX) / pixD;
                telemetry.addData("blob" + idx,
                        "A=%.0f pixD=%.0f R=%.1f cx=%.0f",
                        b.getContourArea(), pixD, range, box.center.x);
                idx++;
                if (idx >= 4) break; // limit to 4 blobs to avoid telemetry overflow
            }

            telemetry.update();
            sleep(50);
        }

        vision.close();
    }
}
