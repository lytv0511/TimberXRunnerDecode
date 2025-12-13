package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private DcMotorEx launcherFlywheel;
    private DcMotor launcherElevator;

    private VisionPortal visionPortal;
    private DcMotor sorter;
    private CRServo intakeServo;
    private MecanumDrive drive;
    private AprilTagProcessor aprilTag;

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
        launcherFlywheel = hardwareMap.get(DcMotorEx.class, "launcherFlywheel");
        launcherElevator = hardwareMap.get(DcMotor.class, "launcherElevator");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        launcherFlywheel.setDirection(DcMotor.Direction.FORWARD);
        launcherElevator.setDirection(DcMotor.Direction.REVERSE);
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setDirection(DcMotor.Direction.REVERSE);

        drive = new MecanumDrive(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            loopLogic();
        }

        // clean up vision
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

    /**
     * Cancel intake and reset pattern (manual hotkey X)
     * Does NOT touch launcherFlywheel or launcherElevator power.
     */
    private void resetIntakeState() {
        // stop sensing & intake
        sensorActive = false;
        intakeServo.setPower(0);

        // stop sorter and return to pos1
        sorter.setPower(0);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sorter.setPower(0);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // clear pattern and counters
        ballCount = 0;
        storePatternBuilder.setLength(0);
        lastSensorColor = 0;

        telemetry.addData("Stored Pattern", storePatternBuilder.toString());
        telemetry.addData("Balls Loaded", ballCount);
        telemetry.update();
    }

    /**
     * Sensor-driven, interruptible intake sequence.
     */
    private void defaultIntakeSequence() {
        // If another sequence is running, ignore
        if (launcherSequenceBusy) return;
        launcherSequenceBusy = true;

        // prepare
        launcherElevator.setPower(0.2);
        sensorActive = true;
        intakeServo.setPower(-1); // start intake

        boolean initialAState = gamepad1.a; // capture current A state so a subsequent press can cancel
        boolean canceled = false;

        // Ensure sorter starts at pos1
//        sorter.setTargetPosition(0);
//        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sorter.setPower(0.3);
//        while (sorter.isBusy() && opModeIsActive()) { idle(); }
//        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        moveSorterToPos3();

        // main loop: collect up to 3 balls unless interrupted
        int lastHandledBallCount = 0;
        while (opModeIsActive() && ballCount <= 3 && !canceled) {
            // detect manual cancel (X or second A press)
            if (gamepad1.x) {
                canceled = true;
                break;
            }
            boolean curA = gamepad1.a;
            if (curA && !initialAState) {
                // A was pressed again -> cancel
                canceled = true;
                break;
            }

            // read sensor (will update ballCount/storePatternBuilder)
            readColorSensor();

            // if a new ball was detected, move sorter to next pos immediately
            if (ballCount > lastHandledBallCount) {
                if (ballCount == 1) {
                    moveSorterToPos3();
                    while (sorter.isBusy() && opModeIsActive()) { idle(); }
                } else if (ballCount == 2) {
                    moveSorterToPos1();
                    while (sorter.isBusy() && opModeIsActive()) { idle(); }
                } else if (ballCount == 3) {
                    moveSorterToPos2();
                } else if (ballCount >= 4) {
                    break;
                }
                lastHandledBallCount += 1;
            }

            idle();
        }

        // stop intake & sensor
        sensorActive = false;
        intakeServo.setPower(0);
        launcherElevator.setPower(0);

        // If canceled by X (manual reset), clear pattern immediately
        if (gamepad1.x || canceled) {
            ballCount = 0;
            storePatternBuilder.setLength(0);
            lastSensorColor = 0;
        }

        // ensure sorter returns to pos1 (so launcher can read balls)
//        sorter.setTargetPosition(0);
//        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sorter.setPower(0.3);
//        while (sorter.isBusy() && opModeIsActive()) { idle(); }
//        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sorter.setPower(0);
//        moveSorterToPos3();
//        intakeServo.setPower(-1);
//        sleep(3000);
        intakeServo.setPower(0);

        telemetry.addData("Stored Pattern", storePatternBuilder.toString());
        telemetry.addData("Balls Loaded", ballCount);
        telemetry.update();

        launcherSequenceBusy = false;
    }

    /**
     * Default Launch Sequence:
     * flywheel ON → augPos1 → elevator ON → augPos2 → 1 sec → augPos3 → return to pos1
     */
//    private void defaultLaunchSequence() {
//        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        launcherSequenceBusy = true;
//        launcherFlywheel.setVelocity(6000 * 537.7);
//        sleep(3000);
//        sorter.setPower(0);
//        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        launcherElevator.setPower(-1.0);
//
//        // Step 1: augPos3
//        sorter.setTargetPosition((int)augPos3 - 30);
//        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sorter.setPower(0.2);
//        sleep(1500);
//        while (sorter.isBusy() && opModeIsActive()) { idle(); }
//        sleep(2000);
//        launcherElevator.setPower(0.1);
//
//        // Step 2: augPos1
//        sorter.setTargetPosition((int)augPos1 - 30);
//        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sorter.setPower(0.2);
//        sleep(1500);
//        launcherElevator.setPower(-1.0);
//        while (sorter.isBusy() && opModeIsActive()) { idle(); }
//        sleep(2000);
//        launcherElevator.setPower(0.1);
//
//        // Step 3: augPos2
//        sorter.setTargetPosition((int)augPos2 - 30);
//        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sorter.setPower(0.2);
//        sleep(1500);
//        launcherElevator.setPower(-1.0);
//        while (sorter.isBusy() && opModeIsActive()) { idle(); }
//        sleep(2000);
//        launcherElevator.setPower(0.1);
//
//        // Return to pos1
//        sorter.setTargetPosition(0);
//        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sorter.setPower(0.3);
//        while (sorter.isBusy() && opModeIsActive()) { idle(); }
//
//        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sorter.setPower(0);
//        launcherElevator.setPower(0);
//        launcherFlywheel.setPower(0);
//        launcherSequenceBusy = false;
//    }

    private void defaultLaunchSequence() {
        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherSequenceBusy = true;
        launcherFlywheel.setVelocity(6000 * 537.7);
        launcherElevator.setPower(-1);
        sleep(2000);
        sorter.setPower(0);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);

        // Step 1: augPos3
        sorter.setTargetPosition((int)augPos3);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sleep(1200);


        // Step 2: augPos1
        sorter.setTargetPosition((int)augPos1);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sleep(1550);


        // Step 3: augPos2
        sorter.setTargetPosition((int)augPos2);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }


        // Return to pos1
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sorter.setPower(0);
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherElevator.setPower(0);
        launcherFlywheel.setPower(0);
        // clear stored intake pattern after launching
        storePatternBuilder.setLength(0);
        ballCount = 0;
        lastSensorColor = 0;
        launcherSequenceBusy = false;
    }

    //         -------------Previous launcher--------------
    // private void defaultLaunchSequence() {
    //        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //        launcherSequenceBusy = true;
    //        launcherFlywheel.setVelocity(6000 * 537.7);
    //        launcherElevator.setPower(0.1);
    //        sleep(3000 / 2);
    //        sorter.setPower(0);
    //        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //        launcherElevator.setPower(-1.0);
    //
    //        // Step 1: augPos3
    //        sorter.setTargetPosition((int)augPos3);
    //        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //        sorter.setPower(0.2);
    //        sleep(1500 / 2);
    //        while (sorter.isBusy() && opModeIsActive()) { idle(); }
    //        sleep(2000 / 2);
    //        launcherElevator.setPower(0.1);
    //
    //        // Step 2: augPos1
    //        sorter.setTargetPosition((int)augPos1);
    //        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //        sorter.setPower(0.2);
    //        sleep(1500 / 2);
    //        launcherElevator.setPower(-1.0);
    //        while (sorter.isBusy() && opModeIsActive()) { idle(); }
    //        sleep(2000 / 2);
    //        launcherElevator.setPower(0.1);
    //
    //        // Step 3: augPos2
    //        sorter.setTargetPosition((int)augPos2);
    //        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //        sorter.setPower(0.2);
    //        sleep(1500 / 2);
    //        launcherElevator.setPower(-1.0);
    //        while (sorter.isBusy() && opModeIsActive()) { idle(); }
    //        sleep(2000 / 2);
    //        launcherElevator.setPower(0.1);
    //
    //        // Return to pos1
    //        sorter.setTargetPosition(0);
    //        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //        sorter.setPower(0.3);
    //        while (sorter.isBusy() && opModeIsActive()) { idle(); }
    //
    //
    //        sorter.setPower(0);
    //        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //        launcherElevator.setPower(0);
    //        launcherFlywheel.setPower(0);
    //        launcherSequenceBusy = false;
    //    }

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

    private void moveSorterToPos1() {
        double pos2Encoder = 0; // 120°
        sorter.setTargetPosition((int) pos2Encoder);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
    }

    private void moveSorterToPos2() {
        double pos2Encoder = ticksPerRevolution / 3; // 120°
        sorter.setTargetPosition((int) pos2Encoder);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
    }

    private void moveSorterToPos3() {
        double pos3Encoder = (ticksPerRevolution * 2) / 3; // 240°
        sorter.setTargetPosition((int) pos3Encoder);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
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

    private void updatePatternTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        int tagId = -1;
        if (detections != null && !detections.isEmpty()) {
            tagId = detections.get(0).id;
        }

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

        // === APRILTAG CAMERA-RELATIVE TELEMETRY ===
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) {
            telemetry.addLine("No AprilTag detected.");
        } else {
            AprilTagDetection det = detections.get(0);

            double flatDistance = Math.hypot(det.ftcPose.x, det.ftcPose.y) * 1.1;
            double theta = Math.toDegrees(Math.atan2(det.ftcPose.y, det.ftcPose.x));

            telemetry.addLine("=== Direct Camera → Tag ===");
            telemetry.addData("Flat Distance", "%.2f in", flatDistance);
            telemetry.addData("Horizontal Angle θ", "%.2f°", theta);
            telemetry.addData("Tag Yaw", "%.2f°", det.ftcPose.yaw);
            telemetry.addData("Tag Pitch", "%.2f°", det.ftcPose.pitch);
            telemetry.addData("Tag Roll", "%.2f°", det.ftcPose.roll);
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
            defaultIntakeSequence();
        }

        // Manual reset hotkey (X) - cancel intake & clear stored pattern instantly
        if (gamepad1.x) {
            resetIntakeState();
        }

        if (gamepad1.b) {
            defaultLaunchSequence();
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
            launcherElevator.setPower(0.1);
            target = gamepad1.left_bumper ? pos3 : augPos3;
            sleep(500);
            launcherElevator.setPower(0);
        }
        // D-Pad Up
        if (gamepad1.dpad_up) {
            launcherElevator.setPower(0.1);
            target = gamepad1.left_bumper ? pos2 : augPos2;
            sleep(500);
            launcherElevator.setPower(0);
        }
        // D-Pad Left
        if (gamepad1.dpad_left) {
            launcherElevator.setPower(0.1);
            target = gamepad1.left_bumper ? pos1 : augPos1;
            sleep(500);
            launcherElevator.setPower(0);
        }

        // If any D-pad button pressed, move motor
        if (gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_left) {
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sorter.setTargetPosition((int) target);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.3);
        }

        String targetPattern = detectedPattern;
        String storePattern = storePatternBuilder.toString();

        if (gamepad1.dpad_down) {
            switch (storePattern) {
                case "GPP":
                    switch (targetPattern) {
                        case "GPP":
                            initiateLaunchSequence(augPos1, augPos2, augPos3);
                            break;
                        case "PGP":
                            initiateLaunchSequence(augPos2, augPos3, augPos1);
                            break;
                        case "PPG":
                            initiateLaunchSequence(augPos3, augPos2, augPos1);
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
                    }
                    break;
            }
        }

        telemetry.addData("Sorter Position", sorter.getCurrentPosition());
        telemetry.addData("Sorter Target", sorter.getTargetPosition());
        telemetry.addData("Sorter Busy", sorter.isBusy());

        if (!launcherSequenceBusy &&
                sorter.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                !sorter.isBusy()) {
            sorter.setPower(0);
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set both launchers to that power
        launcherFlywheel.setPower(triggerPower);
        if (gamepad1.right_bumper) {
            launcherElevator.setPower(launcherTrigger);
        } else {
            launcherElevator.setPower(-launcherTrigger);
        }

        telemetry.addData("Trigger", triggerPower);
        telemetry.addData("Launcher1 Power", launcherFlywheel.getPower());
        telemetry.addData("Launcher2 Power", launcherElevator.getPower());
        telemetry.addData("Stored Pattern", storePatternBuilder.toString());
        telemetry.addData("Balls Loaded", ballCount);
        telemetry.update();
    }
}