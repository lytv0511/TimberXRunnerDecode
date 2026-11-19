package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Limelight 3A wrapper for BIOBUZZ — AprilTags only.
 *
 * Hardware: Limelight 3A named exactly "limelight" on USB 3.0. Color detection
 * is NOT done here; the Logitech webcam ("webcam", USB 2.0) handles yellow /
 * red / blue pollen & nectar via {@link PollenVision}.
 *
 * Tag IDs, field map and camera calibration live in the Limelight web UI.
 * Odometry is independent: pose still comes from the 2 odometry pods + goBILDA
 * Pinpoint. Limelight results are observations only and are never written into
 * the localizer (AprilTag relocalization is deferred).
 */
public class LimelightVision {

    public static final String DEVICE_NAME = "limelight";

    private final Limelight3A limelight;
    private LLResult lastResult;

    public LimelightVision(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, DEVICE_NAME);
    }

    /** Switch pipeline and start polling. Call once at init. */
    public void start(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    /** Switch AprilTag pipelines at runtime (index configured in StudioAprilTag). */
    public void usePipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    /** Refresh and return the latest result. May be null or invalid. */
    public LLResult poll() {
        lastResult = limelight.getLatestResult();
        return lastResult;
    }

    public boolean hasValidResult() {
        LLResult r = lastResult != null ? lastResult : poll();
        return r != null && r.isValid();
    }

    public LLResult getLastResult() { return lastResult; }
    public LLStatus getStatus() { return limelight.getStatus(); }
    public long getTimeSinceLastUpdateMs() { return limelight.getTimeSinceLastUpdate(); }
    public Limelight3A getDevice() { return limelight; }

    public void stop() {
        limelight.stop();
        lastResult = null;
    }
}
