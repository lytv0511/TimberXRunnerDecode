package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.vision.LimelightVision;

import java.util.Collections;
import java.util.List;

/**
 * AprilTag pose from the Limelight 3A fiducial pipeline.
 *
 * Camera calibration and tag IDs live in the Limelight web UI, not in code.
 * This class only reads results. It does NOT write into the Pinpoint/2-pod
 * localizer — fusing vision into the pose estimate is a separate decision.
 *
 * Odometry remains: 2 odometry pods + goBILDA Pinpoint named "pinpoint".
 */
@Config
public class StudioAprilTag {

    /** Limelight pipeline containing the AprilTag fiducial targets. */
    public static int APRILTAG_PIPELINE = 1;
    public static double MIN_TAG_AREA_PCT = 0.2;
    /** Multi-tag pose (MT2) is usually steadier if the field map is loaded. */
    public static boolean USE_MT2_POSE = false;

    private LimelightVision ll;
    private Position lastPosition = null;
    private YawPitchRollAngles lastOrientation = null;
    private LLResultTypes.FiducialResult lastDetection = null;
    private int tagCount = 0;

    public void init(HardwareMap hw) {
        close();
        ll = new LimelightVision(hw);
        ll.start(APRILTAG_PIPELINE);
    }

    /** Re-apply the configured pipeline after a Dashboard change. */
    public void rebuild() {
        if (ll != null) ll.usePipeline(APRILTAG_PIPELINE);
    }

    /** Refresh results and cache the best robot pose. Call once per loop. */
    public void updatePose() {
        if (ll == null) return;

        LLResult result = ll.poll();
        if (result == null || !result.isValid()) {
            tagCount = 0;
            lastDetection = null;
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        tagCount = result.getBotposeTagCount();

        if (fiducials != null && !fiducials.isEmpty()) {
            LLResultTypes.FiducialResult best = null;
            double bestArea = -1;
            for (LLResultTypes.FiducialResult fr : fiducials) {
                if (fr.getTargetArea() > bestArea) {
                    bestArea = fr.getTargetArea();
                    best = fr;
                }
            }
            if (best != null && bestArea >= MIN_TAG_AREA_PCT) lastDetection = best;
        }

        Pose3D pose = USE_MT2_POSE ? result.getBotpose_MT2() : result.getBotpose();
        if (pose != null && tagCount > 0) {
            lastPosition = pose.getPosition();
            lastOrientation = pose.getOrientation();
        }
    }

    public List<LLResultTypes.FiducialResult> getDetections() {
        if (ll == null) return Collections.emptyList();
        LLResult r = ll.getLastResult();
        return r != null ? r.getFiducialResults() : Collections.emptyList();
    }

    /** Largest visible fiducial, or null. */
    public LLResultTypes.FiducialResult getBestDetection() { return lastDetection; }

    public int getTagCount() { return tagCount; }
    public boolean hasPose() { return lastPosition != null; }
    public Position getRobotPosition() { return lastPosition; }
    public YawPitchRollAngles getRobotOrientation() { return lastOrientation; }

    public String getTagName() {
        return lastDetection != null
                ? "ID " + lastDetection.getFiducialId() + " (" + lastDetection.getFamily() + ")"
                : null;
    }

    public int getTagId() {
        return lastDetection != null ? lastDetection.getFiducialId() : -1;
    }

    /** Lateral offset (x) in inches, clamped to +/- maxLateralOffset. 0 if no pose. */
    public double getClampedX(double maxLateralOffset) {
        if (lastPosition == null) return 0;
        return Math.max(-maxLateralOffset, Math.min(maxLateralOffset, lastPosition.x));
    }

    public double getYawDegrees() {
        return lastOrientation != null ? lastOrientation.getYaw(AngleUnit.DEGREES) : 0;
    }

    public LimelightVision getLimelight() { return ll; }

    public void close() {
        if (ll != null) { ll.stop(); ll = null; }
        lastPosition = null;
        lastOrientation = null;
        lastDetection = null;
        tagCount = 0;
    }

    /** Alias for callers using the old name. */
    public void shutdown() { close(); }
}
