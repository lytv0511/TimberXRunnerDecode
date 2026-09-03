package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Unified Pollen detector for BIOBUZZ (3" yellow balls).
 * Single VisionPortal + single ColorBlobLocatorProcessor.
 *
 * All parameters are @Config — tune via Dashboard without code changes.
 * Supports both SDK preset colors and custom HSV ranges.
 *
 * Calibration workflow (run VisionCalibration TeleOp first):
 * 1. Place Pollen at 12" from camera
 * 2. Set COLOR_MODE = 6 (CUSTOM) if preset doesn't match
 * 3. Tune CUSTOM_H/S/V bounds until contours appear around ball
 * 4. Read pixelDiam from telemetry -> set FOCAL_PX = suggestFocal(pixelDiam, 12)
 * 5. Verify range at 6"/12"/24"/36"
 * 6. Tune MIN_AREA_PX to reject floor glare
 */
@Config
public class PollenVision {

    // ---- Color mode: 0=YELLOW, 1=GREEN, 2=RED, 3=BLUE, 4=ARTIFACT_GREEN, 5=ARTIFACT_PURPLE, 6=CUSTOM_HSV ----
    public static int COLOR_MODE = 0; // default YELLOW for BIOBUZZ pollen

    // ---- Custom HSV bounds (used when COLOR_MODE = 6) ----
    // HSV ranges: H=0-180, S=0-255, V=0-255
    public static int CUSTOM_H_MIN = 15;
    public static int CUSTOM_H_MAX = 35;
    public static int CUSTOM_S_MIN = 100;
    public static int CUSTOM_S_MAX = 255;
    public static int CUSTOM_V_MIN = 100;
    public static int CUSTOM_V_MAX = 255;

    // ---- Detection filtering ----
    public static double MIN_AREA_PX = 350;
    public static double MAX_AREA_PX = 18000;
    public static double MIN_ASPECT_RATIO = 0.4;
    public static double MAX_ASPECT_RATIO = 2.5;
    public static double MIN_SOLIDITY = 0.45;
    public static int BLUR_SIZE = 5;

    // ---- Pinhole model ----
    public static double POLLEN_DIAM_IN = 3.0;
    public static double FOCAL_PX = 560;          // correct for 640x480; calibrate with suggestFocal()
    public static double CAMERA_FOV_DEG = 60;

    private VisionPortal portal;
    private ColorBlobLocatorProcessor blobProc;
    private int cameraWidth = 640;

    // ---- Detection result ----
    public static class Detection {
        public final double cx, cy;
        public final double area;
        public final double pixelDiam;
        public final double rangeIn;
        public final double bearingDeg;
        public final double aspectRatio;
        public final double solidity;
        public final double score;
        public final RotatedRect box;

        public Detection(RotatedRect box, double rangeIn, double bearingDeg,
                         double aspectRatio, double solidity, double score) {
            this.box = box;
            this.cx = box.center.x;
            this.cy = box.center.y;
            this.area = box.size.area();
            this.pixelDiam = Math.max(box.size.width, box.size.height);
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.aspectRatio = aspectRatio;
            this.solidity = solidity;
            this.score = score;
        }

        @Override
        public String toString() {
            return String.format("R=%.1f B=%.1f A=%.0f AR=%.2f S=%.2f Sc=%.0f",
                    rangeIn, bearingDeg, area, aspectRatio, solidity, score);
        }
    }

    private Detection lastBest = null;

    // ---- Initialization ----

    public void init(HardwareMap hw, String camName, Size resolution) {
        this.cameraWidth = resolution.getWidth();

        ColorRange colorRange = buildColorRange();

        blobProc = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(BLUR_SIZE)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, camName))
                .setCameraResolution(resolution)
                .addProcessor(blobProc)
                .build();
        portal.resumeStreaming();
    }

    public void init(HardwareMap hw, String camName) {
        init(hw, camName, new Size(640, 480));
    }

    private ColorRange buildColorRange() {
        return switch (COLOR_MODE) {
            case 0 -> ColorRange.YELLOW;
            case 1 -> ColorRange.GREEN;
            case 2 -> ColorRange.RED;
            case 3 -> ColorRange.BLUE;
            case 4 -> ColorRange.ARTIFACT_GREEN;
            case 5 -> ColorRange.ARTIFACT_PURPLE;
            case 6 -> new ColorRange(
                    ColorSpace.HSV,
                    new Scalar(CUSTOM_H_MIN, CUSTOM_S_MIN, CUSTOM_V_MIN),
                    new Scalar(CUSTOM_H_MAX, CUSTOM_S_MAX, CUSTOM_V_MAX));
            default -> ColorRange.YELLOW;
        };
    }

    // ---- Core: poll once per loop iteration ----

    public Detection poll() {
        if (blobProc == null) return null;

        List<ColorBlobLocatorProcessor.Blob> blobs = blobProc.getBlobs();
        if (blobs.isEmpty()) { lastBest = null; return null; }

        List<ColorBlobLocatorProcessor.Blob> filtered = new ArrayList<>(blobs);

        // Area filter
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                MIN_AREA_PX, MAX_AREA_PX, filtered);

        Detection best = null;
        double bestScore = -1;

        for (ColorBlobLocatorProcessor.Blob b : filtered) {
            RotatedRect box = b.getBoxFit();
            double pixW = box.size.width;
            double pixH = box.size.height;
            double pixD = Math.max(pixW, pixH);

            if (pixD < 6) continue;

            // Aspect ratio (longer/shorter, always >= 1 for SDK getAspectRatio)
            double ar = b.getAspectRatio();
            if (ar < MIN_ASPECT_RATIO || ar > MAX_ASPECT_RATIO) continue;

            // Solidity: how filled the bounding box is (sphere -> ~0.78 for circle in box)
            double contourArea = b.getContourArea();
            double boxArea = pixW * pixH;
            double sol = boxArea > 0 ? contourArea / boxArea : 0;
            if (sol < MIN_SOLIDITY) continue;

            // Pinhole range
            double range = (POLLEN_DIAM_IN * FOCAL_PX) / pixD;

            // Bearing from FOV linear model
            double bearing = ((box.center.x - cameraWidth / 2.0) / (cameraWidth / 2.0))
                    * (CAMERA_FOV_DEG / 2.0);

            // Score: prefer large, central, near, spherical blobs
            double centrality = 1.0 - Math.abs(bearing) / (CAMERA_FOV_DEG / 2.0 + 1e-6);
            double sphericalBonus = 1.0 - Math.abs(ar - 1.0) * 0.5;
            double score = contourArea * 0.4
                    + centrality * 600
                    + (range < 36 ? 400 : 0)
                    + sol * 300
                    + sphericalBonus * 200;

            if (score > bestScore) {
                bestScore = score;
                best = new Detection(box, range, bearing, ar, sol, score);
            }
        }

        lastBest = best;
        return best;
    }

    // ---- Convenience accessors ----

    public boolean hasTarget() { return lastBest != null; }
    public Detection getLast() { return lastBest; }
    public double getBearingDeg() { return lastBest != null ? lastBest.bearingDeg : 0; }
    public double getRangeIn() { return lastBest != null ? lastBest.rangeIn : Double.POSITIVE_INFINITY; }
    public double getCenterX() { return lastBest != null ? lastBest.cx : Double.NaN; }
    public double getArea() { return lastBest != null ? lastBest.area : 0; }

    // ---- Raw blob access for calibration opmode ----

    public List<ColorBlobLocatorProcessor.Blob> getRawBlobs() {
        return blobProc != null ? blobProc.getBlobs() : java.util.Collections.emptyList();
    }

    public VisionPortal getPortal() { return portal; }
    public ColorBlobLocatorProcessor getProcessor() { return blobProc; }

    // ---- Cleanup ----

    public void close() {
        if (portal != null) { portal.close(); portal = null; }
        blobProc = null;
        lastBest = null;
    }

    // ---- Calibration helper ----

    /** Point camera at 3" Pollen at known 12" and adjust FOCAL_PX until getRange()==12. */
    public static double suggestFocal(double measuredPixD, double knownRangeIn) {
        return (measuredPixD * knownRangeIn) / POLLEN_DIAM_IN;
    }
}
