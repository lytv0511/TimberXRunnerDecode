package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class StudioAprilTag {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Localization fields
    private Position lastPosition = null;
    private YawPitchRollAngles lastOrientation = null;
    private AprilTagDetection lastDetection = null;

    public void init(HardwareMap hwMap, String cameraName) {
        // Camera pose relative to bot center
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, cameraName));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        visionPortal.resumeStreaming();
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag != null) {
            return aprilTag.getDetections();
        }
        return java.util.Collections.emptyList();
    }

    public AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = getDetections();
        if (detections == null || detections.isEmpty()) return null;

        for (AprilTagDetection d : detections) {
            if (d.robotPose != null && d.metadata != null) {
                return d;
            }
        }
        // fallback: just return first with a pose
        for (AprilTagDetection d : detections) {
            if (d.robotPose != null) {
                return d;
            }
        }
        return null;
    }

    /**
     * Update the cached pose and detection from the current best detection.
     * Should be called periodically to refresh localization data.
     */
    public void updatePose() {
        AprilTagDetection detection = getBestDetection();
        if (detection != null && detection.robotPose != null) {
            lastDetection = detection;
            lastPosition = detection.robotPose.getPosition();
            lastOrientation = detection.robotPose.getOrientation();
        }
    }

    /**
     * Returns the last known robot position from AprilTag localization.
     * May be null if no valid pose has been detected yet.
     */
    public Position getRobotPosition() {
        return lastPosition;
    }

    /**
     * Returns the last known robot orientation from AprilTag localization.
     * May be null if no valid pose has been detected yet.
     */
    public YawPitchRollAngles getRobotOrientation() {
        return lastOrientation;
    }

    /**
     * Returns the name of the last detected AprilTag, or null if none.
     */
    public String getTagName() {
        if (lastDetection != null && lastDetection.metadata != null) {
            return lastDetection.metadata.name;
        }
        return null;
    }

    /**
     * Returns the ID of the last detected AprilTag, or -1 if none.
     */
    public int getTagId() {
        if (lastDetection != null) {
            return lastDetection.id;
        }
        return -1;
    }

    /**
     * Returns the lateral offset (x) in inches, clamped to a maximum magnitude.
     * Returns 0 if no valid position.
     */
    public double getClampedX(double maxLateralOffset) {
        if (lastPosition == null) return 0;
        double x = lastPosition.x; // access field directly
        if (x > maxLateralOffset) return maxLateralOffset;
        if (x < -maxLateralOffset) return -maxLateralOffset;
        return x;
    }

    /**
     * Returns the yaw angle in degrees, or 0 if no valid orientation.
     */
    public double getYawDegrees() {
        if (lastOrientation == null) return 0;
        return lastOrientation.getYaw(AngleUnit.DEGREES);
    }

    public void shutdown() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTag = null;
        lastPosition = null;
        lastOrientation = null;
        lastDetection = null;
    }
}