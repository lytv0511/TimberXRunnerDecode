package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * BIOBUZZ pollen / nectar detector on the Logitech webcam named {@code "webcam"}.
 *
 * The webcam (USB 2.0) handles color only: yellow pollen, red nectar, blue nectar.
 * The Limelight ({@code "limelight"}, USB 3.0) is AprilTag-only — see
 * {@code StudioAprilTag} / {@code LimelightVision}.
 *
 * One VisionPortal hosts up to three ColorBlobLocatorProcessors (one per enabled
 * color). {@link #poll()} returns the best-scoring blob across all enabled colors;
 * {@link #poll(int)} can restrict the search to one color.
 *
 * Odometry is untouched: pose still comes from the 2 odometry pods + goBILDA
 * Pinpoint. This class only reports bearing + a calibrated range estimate.
 */
@Config
public class PollenVision {

    /** Exact Robot Controller config name of the Logitech webcam. */
    public static final String DEVICE_NAME = "webcam";

    /** Color codes used by Detection.color, poll(int) and the assist's TARGET_COLOR. */
    public static final int YELLOW = 0;
    public static final int RED = 1;
    public static final int BLUE = 2;

    // --- Which colors to look for (disable unused ones to save CPU) ---
    public static boolean ENABLE_YELLOW = true;
    public static boolean ENABLE_RED = true;
    public static boolean ENABLE_BLUE = true;

    // --- Camera / range estimation ---
    public static int RESOLUTION_WIDTH = 640;
    public static int RESOLUTION_HEIGHT = 480;
    public static double TARGET_DIAM_IN = 3.0;   // game piece diameter
    public static double FOCAL_PX = 560;         // calibrate with suggestFocal()
    public static double CAMERA_FOV_DEG = 60;    // horizontal FOV

    // --- Blob filters (a ball is round: aspect ~1, circularity/density high) ---
    public static double MIN_AREA_PX = 400;
    public static double MAX_AREA_PX = 20000;
    public static double MIN_ASPECT_RATIO = 1.0;
    public static double MAX_ASPECT_RATIO = 1.7;
    public static double MIN_CIRCULARITY = 0.60;
    public static double MIN_DENSITY = 0.50;
    public static double MIN_RANGE_IN = 2.0;
    public static double MAX_RANGE_IN = 72.0;
    public static int BLUR_SIZE = 5;

    private HardwareMap hw;
    private Size resolution;
    private VisionPortal portal;
    private final Map<Integer, ColorBlobLocatorProcessor> processors = new LinkedHashMap<>();
    private double halfWidth = 320;

    /** Scratch list reused every frame/color so poll() doesn't allocate each loop. */
    private final List<ColorBlobLocatorProcessor.Blob> scratch = new ArrayList<>();

    private Detection lastBest = null;
    private long lastDetectMs = Long.MAX_VALUE;

    /** One game-piece observation. score is 0..100 (higher = better). */
    public static class Detection {
        public final int color;             // YELLOW / RED / BLUE
        public final double cx, cy;         // image pixels
        public final double area;           // contour area, px
        public final double pixelDiam;      // larger box side, px
        public final double rangeIn;
        public final double bearingDeg;     // +right from image center
        public final double aspectRatio;
        public final double circularity;
        public final double density;
        public final double score;
        public final RotatedRect box;

        public Detection(int color, RotatedRect box, double rangeIn, double bearingDeg,
                         double aspectRatio, double circularity, double density, double score) {
            this.color = color;
            this.box = box;
            this.cx = box.center.x;
            this.cy = box.center.y;
            this.area = box.size.area();
            this.pixelDiam = Math.max(box.size.width, box.size.height);
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.aspectRatio = aspectRatio;
            this.circularity = circularity;
            this.density = density;
            this.score = score;
        }

        @Override
        public String toString() {
            return String.format("%s R=%.1f B=%.1f A=%.0f Sc=%.0f",
                    colorName(color), rangeIn, bearingDeg, area, score);
        }
    }

    // ------------------------------------------------------------------ setup

    public void init(HardwareMap hw) {
        init(hw, new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT));
    }

    public void init(HardwareMap hw, Size resolution) {
        close();
        this.hw = hw;
        this.resolution = resolution;
        this.halfWidth = resolution.getWidth() / 2.0;

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, DEVICE_NAME))
                .setCameraResolution(resolution);

        if (ENABLE_YELLOW) addProcessor(builder, YELLOW, ColorRange.YELLOW);
        if (ENABLE_RED) addProcessor(builder, RED, ColorRange.RED);
        if (ENABLE_BLUE) addProcessor(builder, BLUE, ColorRange.BLUE);

        // Never build a portal with zero processors.
        if (processors.isEmpty()) addProcessor(builder, YELLOW, ColorRange.YELLOW);

        portal = builder.build();
    }

    private void addProcessor(VisionPortal.Builder builder, int color, ColorRange range) {
        ColorBlobLocatorProcessor proc = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(range)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setBlurSize(BLUR_SIZE)
                .build();
        processors.put(color, proc);
        builder.addProcessor(proc);
    }

    /** Close and re-open the portal so Dashboard resolution/filter/color changes apply live. */
    public void rebuild() {
        if (hw == null || resolution == null) return;
        init(hw, resolution);
    }

    // ------------------------------------------------------------------- poll

    /** Best target across all enabled colors, or null. Call once per OpMode loop. */
    public Detection poll() {
        return poll(-1);
    }

    /**
     * Best target of one color, or null.
     *
     * @param colorFilter YELLOW / RED / BLUE, or -1 for any enabled color
     */
    public Detection poll(int colorFilter) {
        if (portal == null) return null;

        Detection best = null;
        double bestScore = -1;

        for (Map.Entry<Integer, ColorBlobLocatorProcessor> entry : processors.entrySet()) {
            int color = entry.getKey();
            if (colorFilter >= 0 && color != colorFilter) continue;

            List<ColorBlobLocatorProcessor.Blob> blobs = entry.getValue().getBlobs();
            scratch.clear();
            if (blobs != null) scratch.addAll(blobs);
            if (scratch.isEmpty()) continue;

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    MIN_AREA_PX, MAX_AREA_PX, scratch);

            for (ColorBlobLocatorProcessor.Blob b : scratch) {
                RotatedRect box = b.getBoxFit();
                if (box == null) continue;

                double pixD = Math.max(box.size.width, box.size.height);
                if (pixD < 6) continue;

                double ar = b.getAspectRatio();       // SDK returns >= 1
                if (ar < MIN_ASPECT_RATIO || ar > MAX_ASPECT_RATIO) continue;

                double circ = b.getCircularity();     // circle ~= 1
                if (circ < MIN_CIRCULARITY) continue;

                double dens = b.getDensity();         // contour / convex hull
                if (dens < MIN_DENSITY) continue;

                double range = (TARGET_DIAM_IN * FOCAL_PX) / pixD;
                if (range < MIN_RANGE_IN || range > MAX_RANGE_IN) continue;

                double halfFov = Math.max(1.0, CAMERA_FOV_DEG / 2.0);
                double bearing = ((box.center.x - halfWidth) / halfWidth) * halfFov;
                if (Math.abs(bearing) > halfFov) continue;

                double centrality = 1.0 - Math.min(1.0, Math.abs(bearing) / halfFov);
                double proximity = 1.0 - Math.min(1.0, range / Math.max(1.0, MAX_RANGE_IN));
                double areaScore = Math.min(1.0, box.size.area() / 4000.0);
                double score = 100.0 * (0.30 * clamp01(circ) + 0.25 * centrality
                        + 0.20 * proximity + 0.15 * areaScore + 0.10 * clamp01(dens));

                if (score > bestScore) {
                    bestScore = score;
                    best = new Detection(color, box, range, bearing, ar, circ, dens, score);
                }
            }
        }

        lastBest = best;
        if (best != null) lastDetectMs = System.currentTimeMillis();
        return best;
    }

    // ------------------------------------------------------------- accessors

    public boolean hasTarget() { return lastBest != null; }
    public Detection getLast() { return lastBest; }
    public double getBearingDeg() { return lastBest != null ? lastBest.bearingDeg : 0; }
    public double getRangeIn() { return lastBest != null ? lastBest.rangeIn : Double.POSITIVE_INFINITY; }
    public double getArea() { return lastBest != null ? lastBest.area : 0; }

    /** Age of the most recent detection; Long.MAX_VALUE if none yet. */
    public long getLastDetectionAgeMs() {
        return lastBest == null ? Long.MAX_VALUE : System.currentTimeMillis() - lastDetectMs;
    }

    /** Raw, unfiltered blobs for one color (for VisionCalibration). */
    public List<ColorBlobLocatorProcessor.Blob> getRawBlobs(int color) {
        ColorBlobLocatorProcessor p = processors.get(color);
        return p != null ? p.getBlobs() : Collections.emptyList();
    }

    public int getEnabledColorCount() { return processors.size(); }
    public VisionPortal getPortal() { return portal; }

    public void close() {
        if (portal != null) { portal.close(); portal = null; }
        processors.clear();
        lastBest = null;
        lastDetectMs = Long.MAX_VALUE;
        scratch.clear();
    }

    // -------------------------------------------------------- calibration helpers

    /** FOCAL_PX that makes a game piece at knownRangeIn measure measuredPixD. */
    public static double suggestFocal(double measuredPixD, double knownRangeIn) {
        return (measuredPixD * knownRangeIn) / TARGET_DIAM_IN;
    }

    public static String colorName(int color) {
        switch (color) {
            case RED: return "RED";
            case BLUE: return "BLUE";
            case YELLOW:
            default: return "YELLOW";
        }
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
