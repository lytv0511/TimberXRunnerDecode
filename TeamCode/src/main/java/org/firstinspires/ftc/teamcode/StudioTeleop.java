package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="StudioTeleop", group="TeleOp")
public class StudioTeleop extends LinearOpMode {

    // === Intake/Sorter/Pattern state ===
    private boolean sensorActive = false;
    private int ballCount = 0;
    private StringBuilder storePatternBuilder = new StringBuilder();
    private int lastSensorColor = 0;  // 0 = no ball, 1 = green, 2 = purple

    private ColorSensor colorSensor;
    private double ticksPerRevolution = 537.7; // Assuming Neverest 20/40 motor ticks per rev

    // Launcher positions (initialized in runOpMode() or can keep these as default values)
    private double augPos1 = ticksPerRevolution / 2;      // 180°
    private double augPos2 = (ticksPerRevolution * 5) / 6; // 300°
    private double augPos3 = ticksPerRevolution / 6;      // 60°

    private DcMotor launcherFlywheel;
    private DcMotor launcherElevator;
    private DcMotor sorter;
    private CRServo intakeServo;
    private MecanumDrive drive;
    private StudioAprilTag aprilTag;

    private VisionPortal visionPortalDirect;
    private AprilTagProcessor aprilTagDirect;

    private String detectedPattern = "Unknown";

    private static class DirectTagInfo {
        int id;
        double flatDistance;
        double theta;
        double yaw, pitch, roll;
    }

    private double computePowerFromDistance(double d) {
        double minPower = 0.7;
        double maxPower = 1.0;

        double d0 = 60;   // minimum realistic distance (in)
        double d1 = 100;  // maximum realistic distance (in)

        // Clamp distance to avoid log errors
        d = Math.max(d0, Math.min(d, d1));

        double scaled = Math.log(d / d0) / Math.log(d1 / d0);
        scaled = Math.max(0, Math.min(1, scaled));

        return minPower + (maxPower - minPower) * scaled;
    }

    @Override
    public void runOpMode() {
        // Map motors from the configuration
        launcherFlywheel = hardwareMap.get(DcMotor.class, "launcherFlywheel");
        launcherElevator = hardwareMap.get(DcMotor.class, "launcherElevator");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        launcherFlywheel.setDirection(DcMotor.Direction.FORWARD);
        launcherElevator.setDirection(DcMotor.Direction.REVERSE);
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive = new MecanumDrive(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));

        aprilTag = new StudioAprilTag();
        aprilTag.init(hardwareMap, "Webcam 1");

        aprilTagDirect = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .build();

        visionPortalDirect = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagDirect)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            loopLogic();
        }

        // clean up vision
        if (visionPortalDirect != null) {
            visionPortalDirect.close();
        }
    }

    boolean launcherSequenceBusy = false;

    private void initiateLaunchSequence(double targetAugPos1, double targetAugPos2, double targetAugPos3) {
        launcherSequenceBusy = true;
        launcherFlywheel.setPower(1.0);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos1);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos2);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos3);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);
        launcherFlywheel.setPower(0);
        launcherSequenceBusy = false;
    }

    // === Intake/Sorter/Pattern helper methods ===
    private void readColorSensor() {
        if (!sensorActive || ballCount >= 3) return;

        int colorId = readBallColor();

        if (colorId != 0 && colorId != lastSensorColor) {
            char c = (colorId == 1) ? 'G' : 'P';
            storePatternBuilder.append(c);
            ballCount++;
            lastSensorColor = colorId;
        }

        if (colorId == 0) {
            lastSensorColor = 0;
        }
    }

    private int readBallColor() {
        // Returns: 0 = no ball, 1 = green, 2 = purple
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int threshold = 30; // minimum total brightness to consider a ball present

        if (r + g + b < threshold) return 0; // no ball

        if (g > r && g > b) return 1; // green
        if (r > g && b > g) return 2; // purple

        return 0; // default to no ball if ambiguous
    }

    private void moveSorterToPos2() {
        double pos2Encoder = ticksPerRevolution / 3; // 120°
        sorter.setTargetPosition((int) pos2Encoder);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
    }

    private void moveSorterToPos3() {
        double pos3Encoder = (ticksPerRevolution * 2) / 3; // 240°
        sorter.setTargetPosition((int) pos3Encoder);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
    }

    private void performLaunchSequence(String storePattern, String targetPattern) {
        sensorActive = false;  // ensure sensor doesn't interfere

        switch (storePattern) {
            case "GPP":
                switch (targetPattern) {
                    case "GPP":
                        initiateLaunchSequence(augPos1, augPos2, augPos3);
                        break;
                    case "PGP":
                        initiateLaunchSequence(augPos2, augPos1, augPos3);
                        break;
                    case "PPG":
                        initiateLaunchSequence(augPos3, augPos2, augPos1);
                        break;
                    default:
                        initiateLaunchSequence(augPos1, augPos2, augPos3);
                        break;
                }
                break;

            case "PGP":
                switch (targetPattern) {
                    case "GPP":
                        initiateLaunchSequence(augPos2, augPos1, augPos3);
                        break;
                    case "PGP":
                        initiateLaunchSequence(augPos1, augPos2, augPos3);
                        break;
                    case "PPG":
                        initiateLaunchSequence(augPos3, augPos1, augPos2);
                        break;
                    default:
                        initiateLaunchSequence(augPos1, augPos2, augPos3);
                        break;
                }
                break;

            case "PPG":
                switch (targetPattern) {
                    case "GPP":
                        initiateLaunchSequence(augPos3, augPos2, augPos1);
                        break;
                    case "PGP":
                        initiateLaunchSequence(augPos1, augPos3, augPos2);
                        break;
                    case "PPG":
                        initiateLaunchSequence(augPos1, augPos2, augPos3);
                        break;
                    default:
                        initiateLaunchSequence(augPos1, augPos2, augPos3);
                        break;
                }
                break;

            default:
                initiateLaunchSequence(augPos1, augPos2, augPos3);
                break;
        }

        // reset sorter to pos1
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorActive = true; // ready for next intake
    }

    public void artifactPipline() {
        // Step 1: reset sorter to pos1
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }

        // Step 1b: clear sequence
        storePatternBuilder.setLength(0);
        ballCount = 0;
        lastSensorColor = 0;

        // Step 2: activate sensor and intake
        sensorActive = true;
        intakeServo.setPower(1);

        // Step 3: loop until all 3 balls are detected
        while (ballCount < 3 && opModeIsActive()) {
            readColorSensor();

            // Move sorter to next position after each ball detected
            if (ballCount == 1) moveSorterToPos2();
            else if (ballCount == 2) moveSorterToPos3();

            idle();
        }

        // Step 4: deactivate sensor and stop intake
        sensorActive = false;
        intakeServo.setPower(0);

        // Step 5: feed sequence to launch
        String storePattern = storePatternBuilder.toString();
        performLaunchSequence(storePattern, detectedPattern);

        // Step 6: reset for next intake
        ballCount = 0;
        storePatternBuilder.setLength(0);
        sorter.setTargetPosition(0);  // return to pos1
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private DirectTagInfo getDirectTag() {
        List<AprilTagDetection> detections = aprilTagDirect.getDetections();
        if (detections == null || detections.isEmpty()) return null;

        AprilTagDetection det = detections.get(0);

        double x = det.ftcPose.x;
        double y = det.ftcPose.y;
        double z = det.ftcPose.z;

        DirectTagInfo info = new DirectTagInfo();
        info.id = det.id;
        info.flatDistance = Math.hypot(x, y) * 1.1;
        info.theta = Math.toDegrees(Math.atan2(y, x));
        info.yaw = det.ftcPose.yaw;
        info.pitch = det.ftcPose.pitch;
        info.roll = det.ftcPose.roll;

        return info;
    }

    private void updatePatternTag() {
        aprilTag.updatePose();
        int tagId = aprilTag.getTagId();

        switch (tagId) {
            case 21: detectedPattern = "GPP"; break;
            case 22: detectedPattern = "PGP"; break;
            case 23: detectedPattern = "PPG"; break;
            default: /* leave detectedPattern unchanged if unknown */ break;
        }
    }

    private void loopLogic() {
        updatePatternTag();
        telemetry.addData("Game Pattern", detectedPattern);

        double driveY = -gamepad1.left_stick_y;
        double driveX = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                new com.acmerobotics.roadrunner.Vector2d(driveY, driveX),
                turn
        ));

        telemetry.addData("DriveY", driveY);
        telemetry.addData("DriveX", driveX);
        telemetry.addData("Turn", turn);

        // === APRILTAG LOCALIZATION & TELEMETRY ===
        aprilTag.updatePose();

        int tagId = aprilTag.getTagId();
        String tagName = aprilTag.getTagName();
        double yaw = aprilTag.getYawDegrees();
        org.firstinspires.ftc.robotcore.external.navigation.Position pos = aprilTag.getRobotPosition();

        telemetry.addLine("=== AprilTag Detection (TeleOp) ===");
        double distance = 0;

        // get direct camera detection once per loop to avoid duplicate work
        DirectTagInfo direct = getDirectTag();

        if (tagId != -1) {
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Tag Name", tagName != null ? tagName : "Unknown");
            if (pos != null) {
                double x = pos.x;
                double yPos = pos.y;
                double z = pos.z;

                telemetry.addData("X (in)", "%.2f", x);
                telemetry.addData("Y (in)", "%.2f", yPos);
                telemetry.addData("Z (in)", "%.2f", z);

                // Calculate distance ignoring yPos — prefer direct camera reading when available
                if (direct != null && direct.id == 24) {
                    // prefer direct camera flat-distance (ignores vertical z)
                    distance = direct.flatDistance;
                } else {
                    // fallback to legacy robot-position-based distance (only if no direct detection)
                    distance = Math.sqrt(x * x + z * z);
                }

                telemetry.addData("Distance (in)", "%.2f", distance);

            } else {
                telemetry.addLine("No valid pose detected.");
            }
            telemetry.addData("Yaw (deg)", "%.2f", yaw);
        } else {
            telemetry.addLine("No AprilTag detected.");
        }

        if (direct != null) {
            telemetry.addLine("=== Direct Camera (Auto Integration) ===");
            telemetry.addData("Flat Distance", "%.2f in", direct.flatDistance);
            telemetry.addData("Theta", "%.2f°", direct.theta);
            telemetry.addData("Yaw", "%.2f°", direct.yaw);
            telemetry.addData("Pitch", "%.2f°", direct.pitch);
            telemetry.addData("Roll", "%.2f°", direct.roll);

            if (gamepad1.right_stick_button && direct.id == 24) {
                launcherFlywheel.setPower(computePowerFromDistance(direct.flatDistance));
            }
        }

        // Right trigger on gamepad1 goes from 0.0 to 1.0
        double triggerPower = 0;
        double launcherTrigger = 0;

        if (gamepad1.right_bumper) {
            intakeServo.setPower(1);
        } else if (gamepad1.left_bumper) {
            intakeServo.setPower(0);
        }

        launcherTrigger = gamepad1.right_trigger;
        triggerPower = gamepad1.left_trigger;

        if (gamepad1.a) {
            triggerPower = 0.8;
        }

        if (gamepad1.b) {
            triggerPower = 1.0;
        }

        if (gamepad1.x && direct != null && direct.id == 24) {
            launcherFlywheel.setPower(computePowerFromDistance(direct.flatDistance));
        }

        if (gamepad1.y) {
            triggerPower = 0.75;
        }

        double ticksPerRevolution = 537.7;

        // Calculate positions
        double augPos3 = ticksPerRevolution / 6; // 60°
        double pos2 = ticksPerRevolution / 3; // 120°
        double augPos1 = ticksPerRevolution / 2; // 180°
        double pos3 = (ticksPerRevolution * 2) / 3; // 240°
        double augPos2 = (ticksPerRevolution * 5) / 6; // 300°
        double pos1 = 0; // 360°

        double target = 0;

        // D-Pad Right
        if (gamepad1.dpad_right) {
            target = gamepad1.left_bumper ? pos3 : augPos3;
        }
        // D-Pad Up
        if (gamepad1.dpad_up) {
            target = gamepad1.left_bumper ? pos2 : augPos2;
        }
        // D-Pad Left
        if (gamepad1.dpad_left) {
            target = gamepad1.left_bumper ? pos1 : augPos1;
        }

        // If any D-pad button pressed, move motor
        if (gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_left) {
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sorter.setTargetPosition((int) target);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.5);
        }

        String targetPattern = detectedPattern;
        String storePattern = storePatternBuilder.toString();

//        if (gamepad1.dpad_down) {
//            switch (storePattern) {
//                case "GPP":
//                    switch (targetPattern) {
//                        case "GPP":
//                            initiateLaunchSequence(augPos1, augPos2, augPos3);
//                            break;
//                        case "PGP":
//                            initiateLaunchSequence(augPos2, augPos3, augPos1);
//                            break;
//                        case "PPG":
//                            initiateLaunchSequence(augPos3, augPos2, augPos1);
//                            break;
//                    }
//                    break;
//
//                case "PGP":
//                    switch (targetPattern) {
//                        case "GPP":
//                            initiateLaunchSequence(augPos2, augPos1, augPos3);
//                            break;
//                        case "PGP":
//                            initiateLaunchSequence(augPos1, augPos2, augPos3);
//                            break;
//                        case "PPG":
//                            initiateLaunchSequence(augPos3, augPos1, augPos2);
//                            break;
//                    }
//                    break;
//
//                case "PPG":
//                    switch (targetPattern) {
//                        case "GPP":
//                            initiateLaunchSequence(augPos3, augPos2, augPos1);
//                            break;
//                        case "PGP":
//                            initiateLaunchSequence(augPos1, augPos3, augPos2);
//                            break;
//                        case "PPG":
//                            initiateLaunchSequence(augPos1, augPos2, augPos3);
//                            break;
//                    }
//                    break;
//            }
//        }

        telemetry.addData("Sorter Position", sorter.getCurrentPosition());
        telemetry.addData("Sorter Target", sorter.getTargetPosition());
        telemetry.addData("Sorter Busy", sorter.isBusy());

        if (sorter.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !sorter.isBusy()) {
            sorter.setPower(0);
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set both launchers to that power
        launcherFlywheel.setPower(triggerPower);
        launcherElevator.setPower(-launcherTrigger);

        telemetry.addData("Trigger", triggerPower);
        telemetry.addData("Launcher1 Power", launcherFlywheel.getPower());
        telemetry.addData("Launcher2 Power", launcherElevator.getPower());
        // (Rest of telemetry and loop logic unchanged)
    }
}
