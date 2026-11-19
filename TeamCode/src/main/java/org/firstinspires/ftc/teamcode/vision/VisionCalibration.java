package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.StudioAprilTag;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.RotatedRect;

import java.util.List;

/**
 * BIOBUZZ two-device vision calibration / live viewer.
 *
 *   VIEW = 0  webcam  ("webcam", USB 2.0)   -> yellow / red / blue pollen & nectar
 *   VIEW = 1  Limelight ("limelight", USB 3.0) -> AprilTag fiducials + botpose
 *
 * Webcam color thresholds are SDK presets in code (tunable via the minimum
 * and maximum filter fields on PollenVision). Limelight tag IDs, field map
 * and camera calibration live in the Limelight web UI.
 *
 * AprilTag relocalization is deliberately NOT performed here: the 2 odometry
 * pods + goBILDA Pinpoint remain the only pose source.
 */
@Config
@TeleOp(name = "VisionCalibration", group = "Calibration")
public class VisionCalibration extends LinearOpMode {

    /** 0 = webcam pollen colors, 1 = Limelight AprilTags. */
    public static int VIEW = 0;
    /** Place a game piece at this distance from the webcam. */
    public static double CALIBRATE_DISTANCE_IN = 12.0;
    /** Toggle 0->1 (then back) in Dashboard to re-open the webcam portal. */
    public static double REBUILD = 0;

    private PollenVision pollen;
    private LimelightVision ll;
    private boolean webcamOk = false;
    private boolean limelightOk = false;

    @Override
    public void runOpMode() {
        try {
            pollen = new PollenVision();
            pollen.init(hardwareMap);
            webcamOk = true;
        } catch (RuntimeException e) {
            webcamOk = false;
        }

        try {
            ll = new LimelightVision(hardwareMap);
            ll.start(StudioAprilTag.APRILTAG_PIPELINE);
            limelightOk = true;
        } catch (RuntimeException e) {
            limelightOk = false;
        }

        telemetry.addLine("=== BIOBUZZ Vision Calibration ===");
        telemetry.addData("VIEW", "0=webcam colors, 1=Limelight tags");
        telemetry.addData("webcam", webcamOk ? "OK" : "MISSING (\"webcam\")");
        telemetry.addData("limelight", limelightOk ? "OK" : "MISSING (\"limelight\")");
        telemetry.addData("Calib dist", "%.1f in", CALIBRATE_DISTANCE_IN);
        telemetry.update();

        waitForStart();

        double lastRebuild = REBUILD;

        while (opModeIsActive()) {
            if (REBUILD != lastRebuild) {
                lastRebuild = REBUILD;
                if (webcamOk) pollen.rebuild();
            }

            if (VIEW == 0) {
                showWebcam();
            } else {
                showLimelight();
            }

            telemetry.update();
            sleep(50);
        }

        if (webcamOk) pollen.close();
        if (limelightOk) ll.stop();
    }

    private void showWebcam() {
        if (!webcamOk) {
            telemetry.addLine("--- WEBCAM \"webcam\" NOT CONFIGURED ---");
            return;
        }

        PollenVision.Detection d = pollen.poll();

        telemetry.addLine("--- WEBCAM \"" + PollenVision.DEVICE_NAME + "\" (colors) ---");
        telemetry.addData("Enabled", "Y=%b R=%b B=%b (%d proc)",
                PollenVision.ENABLE_YELLOW, PollenVision.ENABLE_RED,
                PollenVision.ENABLE_BLUE, pollen.getEnabledColorCount());
        telemetry.addData("FOCAL_PX", PollenVision.FOCAL_PX);

        if (d == null) {
            telemetry.addLine("--- NO GAME PIECE DETECTED ---");
            telemetry.addLine("Toggle REBUILD after changing tunables");
        } else {
            telemetry.addLine("--- BEST TARGET ---");
            telemetry.addData("color", PollenVision.colorName(d.color));
            telemetry.addData("range", "%.1f in", d.rangeIn);
            telemetry.addData("bearing", "%.1f deg", d.bearingDeg);
            telemetry.addData("score", "%.0f / 100", d.score);
            telemetry.addData("pixD / area", "%.1f px / %.0f px", d.pixelDiam, d.area);
            telemetry.addData("aspect / circ / dens", "%.2f / %.2f / %.2f",
                    d.aspectRatio, d.circularity, d.density);
            double suggested = PollenVision.suggestFocal(d.pixelDiam, CALIBRATE_DISTANCE_IN);
            telemetry.addData("suggestFocal", "%.0f (piece at %.0f in)",
                    suggested, CALIBRATE_DISTANCE_IN);
        }

        int idx = 0;
        int[] colors = {PollenVision.YELLOW, PollenVision.RED, PollenVision.BLUE};
        for (int color : colors) {
            List<ColorBlobLocatorProcessor.Blob> blobs = pollen.getRawBlobs(color);
            telemetry.addData(PollenVision.colorName(color) + " blobs", blobs.size());
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect box = b.getBoxFit();
                double pixD = Math.max(box.size.width, box.size.height);
                telemetry.addData("  b" + idx, "A=%.0f D=%.0f x=%.0f AR=%.2f C=%.2f",
                        b.getContourArea(), pixD, box.center.x,
                        b.getAspectRatio(), b.getCircularity());
                if (++idx >= 3) break; // keep telemetry under the DS limit
            }
            if (idx >= 3) break;
        }
    }

    private void showLimelight() {
        if (!limelightOk) {
            telemetry.addLine("--- LIMELIGHT \"limelight\" NOT CONFIGURED ---");
            return;
        }

        LLResult result = ll.poll();
        LLStatus status = ll.getStatus();

        telemetry.addLine("--- LIMELIGHT \"" + LimelightVision.DEVICE_NAME + "\" (tags only) ---");
        if (status != null) {
            telemetry.addData("LL", "%s  %.1fC  %.0f%%CPU  %dFPS",
                    status.getName(), status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "%d (%s)",
                    status.getPipelineIndex(), status.getPipelineType());
        }
        telemetry.addData("Staleness", "%d ms", ll.getTimeSinceLastUpdateMs());
        telemetry.addLine("Pose source: Pinpoint 2-pod (no relocalization)");

        if (result == null || !result.isValid()) {
            telemetry.addLine("--- NO VALID LIMELIGHT RESULT ---");
            return;
        }

        telemetry.addData("tx / ty", "%.2f / %.2f deg", result.getTx(), result.getTy());
        telemetry.addData("Tags in pose", result.getBotposeTagCount());

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            telemetry.addLine("--- NO APRILTAGS VISIBLE ---");
            return;
        }

        int idx = 0;
        for (LLResultTypes.FiducialResult f : tags) {
            telemetry.addData("tag" + idx, "ID %d (%s) area=%.2f%%",
                    f.getFiducialId(), f.getFamily(), f.getTargetArea());
            if (++idx >= 4) break;
        }
        if (result.getBotpose() != null) {
            telemetry.addData("botpose", "%s", result.getBotpose().getPosition().toString());
        }
    }
}
