package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * Vision calibration tool for DECODE/BIOBUZZ ball detection.
 * Run this TeleOp on the robot with test balls (green/purple for DECODE, yellow for BIOBUZZ)
 * at known distances.
 *
 * DECODE-season balls only (green + purple artifacts):
 *   Default COLOR_MODE = 4 (ARTIFACT_GREEN) — SDK-tuned for the green artifact ball.
 *   For purple, set COLOR_MODE = 5 (ARTIFACT_PURPLE).
 *   These presets are exact YCrCb ranges from the FTC SDK and need no manual HSV.
 *
 * Workflow (no real yellow balls yet — use green/purple to verify the pipeline):
 * 1. Place a ball at CALIBRATE_DISTANCE (12") from camera
 * 2. Set COLOR_MODE to match the ball's color (4=green, 5=purple)
 * 3. Toggle REBUILD=1 then back to 0 in Dashboard to apply the color change live
 * 4. Read "pixD" from telemetry → set FOCAL_PX = suggested value (rough: 560 for 640x480)
 * 5. Verify "range" ≈ actual distance at 6"/12"/24"/36"
 * 6. Tune MIN_AREA_PX (raise if floor glare detected), MIN_SOLIDITY (raise if shadows pass)
 *
 * Dashboard fields (all @Config):
 *   COLOR_MODE            — 0=YELLOW 1=GREEN 2=RED 3=BLUE 4=ARTIFACT_GREEN 5=ARTIFACT_PURPLE 6=CUSTOM
 *   CUSTOM_H/S/V          — custom HSV bounds (COLOR_MODE=6)
 *   REBUILD (0/1)         — toggle to re-apply COLOR_MODE/HSV/filter changes live (no restart)
 *   FOCAL_PX              — pinhole focal length (560 = rough 640x480 default)
 *   CAMERA_FOV_DEG        — camera horizontal FOV
 *   MIN_AREA_PX, MIN_SOLIDITY, MIN/MAX_ASPECT_RATIO — filters
 */
@Config
@TeleOp(name = "VisionCalibration", group = "Calibration")
public class VisionCalibration extends LinearOpMode {

    // Reference distance for focal calibration — place ball at this distance
    public static double CALIBRATE_DISTANCE_IN = 12.0;
    // Toggle 0->1 (then back to 0) in Dashboard to re-apply color/filter changes live
    public static double REBUILD = 0;

    @Override
    public void runOpMode() {
        PollenVision vision = new PollenVision();
        vision.init(hardwareMap, "Webcam 1");

        telemetry.addLine("=== VisionCalibration ===");
        telemetry.addLine("Place a ball at CALIBRATE_DISTANCE from camera");
        telemetry.addData("Calib dist", CALIBRATE_DISTANCE_IN + " in");
        telemetry.addLine("COLOR_MODE 4=green 5=purple; toggle REBUILD to apply");
        telemetry.update();

        waitForStart();

        double lastRebuild = REBUILD;

        while (opModeIsActive()) {
            // Hot-reload when user toggles REBUILD in Dashboard
            if (REBUILD != lastRebuild) {
                lastRebuild = REBUILD;
                telemetry.addLine("Rebuilding processor with new color/filter...");
                telemetry.update();
                vision.rebuild(hardwareMap, "Webcam 1",
                        new android.util.Size(640, 480));
            }

            PollenVision.Detection d = vision.poll();

            // === Raw blobs (before scoring, for debugging) ===
            List<ColorBlobLocatorProcessor.Blob> raw = vision.getRawBlobs();
            telemetry.addData("Raw blobs", raw.size());
            telemetry.addData("Color mode", PollenVision.colorModeName() + " (" + PollenVision.COLOR_MODE + ")");
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
                telemetry.addLine("Set COLOR_MODE (4=green/5=purple), toggle REBUILD");
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

