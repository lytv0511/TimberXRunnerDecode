package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name="StudioRunnerAuto", group = "Autonomous")
public class StudioRunnerAuto extends LinearOpMode {
    private StudioAprilTag studioAprilTag;

    // Hardware declarations
    private com.qualcomm.robotcore.hardware.DcMotorEx launcherFlywheel;
    private com.qualcomm.robotcore.hardware.DcMotor launcherElevator;
    private com.qualcomm.robotcore.hardware.DcMotor sorter;
    private com.qualcomm.robotcore.hardware.CRServo intakeServo;

    // Shared intake/sorter state copied from StudioTeleop
    private boolean sensorActive = false;
    private int ballCount = 0;
    private StringBuilder storePatternBuilder = new StringBuilder();
    private int lastSensorColor = 0;
    private String detectedPattern = "Unknown";

    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;

    private double ticksPerRevolution = 537.7;
    private double augPos1, augPos2, augPos3;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");

        // Hardware initialization
        launcherFlywheel = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "launcherFlywheel");
        launcherElevator = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "launcherElevator");
        sorter = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "sorter");
        intakeServo = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "intakeServo");
        colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE);

        augPos1 = ticksPerRevolution / 2;
        augPos2 = (ticksPerRevolution * 5) / 6;
        augPos3 = ticksPerRevolution / 6;

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        int tagId;
        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = studioAprilTag.getDetections();
            studioAprilTag.updatePose();

            if (!detections.isEmpty()) {
                tagId = detections.get(0).id;
                telemetry.addData("Detected Tag ID", tagId);

                Position pos = studioAprilTag.getRobotPosition();
                YawPitchRollAngles ori = studioAprilTag.getRobotOrientation();

                if (pos != null && ori != null) {
                    telemetry.addData("Robot X", "%.2f", pos.x);
                    telemetry.addData("Robot Y", "%.2f", pos.y);
                    telemetry.addData("Robot Z", "%.2f", pos.z);
                    telemetry.addData("Robot Yaw", "%.2f", ori.getYaw(AngleUnit.DEGREES));
                    telemetry.addData("Robot Pitch", "%.2f", ori.getPitch(AngleUnit.DEGREES));
                    telemetry.addData("Robot Roll", "%.2f", ori.getRoll(AngleUnit.DEGREES));
                }

            } else {
                telemetry.addLine("No tag detected");
            }
            telemetry.update();
            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .setTangent(Math.toRadians(0))   // heading right (x-positive)
                        .strafeTo(new Vector2d(0, 10))  // go left/right based on field coords
                        .setTangent(Math.toRadians(180)) // now we want to go backwards toward -X
                        .lineToX(-40)                    // RR now has a valid heading
                        .build()
        );

        sleep(500);

        defaultLaunchSequence();

        Pose2d currentPos = drive.localizer.getPose();
        Actions.runBlocking(
                drive.actionBuilder(currentPos)
                        .turn(Math.toRadians(45))
                        .lineToY(50)
                        .turn(Math.toRadians(-45))
                        .build()
        );

//        tagId = detectTagID();
//        telemetry.addData("Detected Tag ID after scanning", tagId);
//
//        String tagName = studioAprilTag.getTagName();
//        if (tagName != null) {
//            telemetry.addData("Tag Name", tagName);
//        }
//
//        Position pos = studioAprilTag.getRobotPosition();
//        YawPitchRollAngles ori = studioAprilTag.getRobotOrientation();
//        if (pos != null && ori != null) {
//            telemetry.addData("Robot X", "%.2f", pos.x);
//            telemetry.addData("Robot Y", "%.2f", pos.y);
//            telemetry.addData("Robot Z", "%.2f", pos.z);
//            telemetry.addData("Robot Yaw", "%.2f", ori.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Robot Pitch", "%.2f", ori.getPitch(AngleUnit.DEGREES));
//            telemetry.addData("Robot Roll", "%.2f", ori.getRoll(AngleUnit.DEGREES));
//        }
//        telemetry.update();
//
//        Action trajectoryActionChosen;
//        Pose2d currentPose = drive.localizer.getPose();
//        if (tagId == 21) {
//            trajectoryActionChosen = drive.actionBuilder(currentPose)
//                    .splineTo(new Vector2d(71, 0), Math.toRadians(-90))
//                    .lineToY(-48)
//                    .waitSeconds(0.5)
//                    .splineTo(new Vector2d(95 + 8, -40 + 8), Math.toRadians(-62))
//                    .build();
//        } else if (tagId == 22) {
//            trajectoryActionChosen = drive.actionBuilder(currentPose)
//                    .lineToX(36)
//                    .strafeTo(new Vector2d(36, 48))
//                    .build();
//        } else if (tagId == 23) {
//            trajectoryActionChosen = drive.actionBuilder(currentPose)
//                    .strafeTo(new Vector2d(46, 30))
//                    .build();
//        } else {
//            trajectoryActionChosen = drive.actionBuilder(currentPose)
//                    .waitSeconds(0.1)
//                    .build();
//        }
//
//        Actions.runBlocking(
//                drive.actionBuilder(currentPose)
//                        .lineToX(103)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(-32)
//                        .turn(Math.toRadians(28))
//                        .build()
//        );
//        sleep(5000);
//        Actions.runBlocking(trajectoryActionChosen);

        studioAprilTag.shutdown();
    }

    private int detectTagID() {
        List<AprilTagDetection> detections = studioAprilTag.getDetections();
        if (!detections.isEmpty()) return detections.get(0).id;
        return -1;
    }

    // === Mirrored methods from TeleOp ===
    private boolean launcherSequenceBusy = false;

    private void initiateLaunchSequence(double targetAugPos1, double targetAugPos2, double targetAugPos3) {
        launcherSequenceBusy = true;
        launcherFlywheel.setPower(1.0);

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos1);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos2);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos3);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);

        sorter.setTargetPosition(0);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setPower(0);
        launcherElevator.setPower(0);
        launcherFlywheel.setPower(0);
        launcherSequenceBusy = false;
    }

private void defaultIntakeSequence() {
    launcherSequenceBusy = true;

    // Reset sorter to pos1
    sorter.setPower(0);
    sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
    sorter.setTargetPosition(0);
    sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
    sorter.setPower(0.4);
    while (sorter.isBusy() && opModeIsActive()) { idle(); }
    sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

    // Reset counters
    ballCount = 0;
    storePatternBuilder.setLength(0);
    lastSensorColor = 0;

    // Start intake and sensor tracking
    sensorActive = true;
    intakeServo.setPower(1);

    int lastHandled = 0;

    // Main loop until three balls detected
    while (opModeIsActive() && ballCount < 3) {
        readColorSensor();

        // If a ball was detected, rotate sorter
        if (ballCount > lastHandled) {
            if (ballCount == 1) {
                moveSorterToPos2();
                while (sorter.isBusy() && opModeIsActive()) { idle(); }
            } else if (ballCount == 2) {
                moveSorterToPos3();
                while (sorter.isBusy() && opModeIsActive()) { idle(); }
            }
            lastHandled = ballCount;
        }

        idle();
    }

    // Stop intake when full
    sensorActive = false;
    intakeServo.setPower(0);

    // Return sorter to pos1
    sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
    sorter.setTargetPosition(0);
    sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
    sorter.setPower(0.4);
    while (sorter.isBusy() && opModeIsActive()) { idle(); }
    sorter.setPower(0);
    sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

    launcherSequenceBusy = false;
}

    private void defaultLaunchSequence() {
        launcherSequenceBusy = true;

        // --- Configure flywheel PIDF ---
        launcherFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherFlywheel.setPIDFCoefficients(
                launcherFlywheel.getMode(), new PIDFCoefficients(10, 0, 0, 15));

        double targetVelocity = 53770; // ticks/sec, adjust as needed
        launcherFlywheel.setVelocity(targetVelocity);

        // --- Ball positions ---
        double[] augPositions = {augPos3, augPos1 - 30, augPos2};

        for (double augPos : augPositions) {
            // Wait for flywheel to reach near target speed
            // **FIX 1: Increased max wait time (2.0s -> 3.0s)**
            ElapsedTime spinTimer = new ElapsedTime();
            spinTimer.reset();
            while (opModeIsActive() &&
                    Math.abs(launcherFlywheel.getVelocity() - targetVelocity) > 1500 &&
                    spinTimer.seconds() < 3.0) { // Increased max wait for stability
                idle();
            }

            // Move sorter to the ball
            sorter.setTargetPosition((int) augPos);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.3); // Increased sorter power slightly (0.2 -> 0.3)

            // Wait for sorter to move
            while (sorter.isBusy() && opModeIsActive()) {
                idle();
            }

            // **FIX 2: Stricter wait for sorter alignment (essential for precision)**
            // Wait until the sorter is within a small tolerance of the target position
            // to ensure alignment is complete before feeding the ball.
            ElapsedTime alignmentTimer = new ElapsedTime();
            alignmentTimer.reset();
            int tolerance = 10; // Adjust this tolerance (in ticks) as needed

            while (opModeIsActive() &&
                    Math.abs(sorter.getCurrentPosition() - (int)augPos) > tolerance &&
                    alignmentTimer.seconds() < 0.5) { // Max wait for fine alignment
                idle();
            }

            // Ensure motor is stopped after alignment for no drift
            sorter.setPower(0);

            // Feed ball using elevator
            launcherElevator.setPower(-1.0); // tuned feed power
            ElapsedTime feedTimer = new ElapsedTime();
            feedTimer.reset();

            // **FIX 3: Increased feed duration (0.5s -> 0.7s)**
            // This ensures a strong, complete ejection of all three balls.
            while (feedTimer.seconds() < 0.7 && opModeIsActive()) {
                idle();
            }
            launcherElevator.setPower(0);
        }

        // Return sorter to position 0
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }

        // Stop all motors safely
        launcherFlywheel.setPower(0);
        launcherElevator.setPower(0);
        sorter.setPower(0);

        launcherSequenceBusy = false;

        // Reset intake counters
        storePatternBuilder.setLength(0);
        ballCount = 0;
        lastSensorColor = 0;
    }

    // === Intake/Sorter/Pattern helper methods ===
    private int readBallColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int threshold = 30;
        if (r+g+b < threshold) return 0;
        if (g > r && g > b) return 1;
        if (r > g && b > g) return 2;
        return 0;
    }

    private void readColorSensor() {
        if (!sensorActive || ballCount >= 3) return;
        int colorId = readBallColor();
        if (colorId != 0 && colorId != lastSensorColor) {
            char c = (colorId == 1) ? 'G' : 'P';
            storePatternBuilder.append(c);
            ballCount++;
            lastSensorColor = colorId;
        }
        if (colorId == 0) lastSensorColor = 0;
    }

    private void moveSorterToPos2() {
        double pos2Enc = ticksPerRevolution / 3;
        sorter.setTargetPosition((int) pos2Enc);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
    }

    private void moveSorterToPos3() {
        double pos3Enc = (ticksPerRevolution * 2) / 3;
        sorter.setTargetPosition((int) pos3Enc);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
    }

    private void performLaunchSequence(String storePattern, String targetPattern) {
        // Minimal stub: choose direct call
        initiateLaunchSequence(augPos1, augPos2, augPos3);
        sorter.setTargetPosition(0);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sensorActive = true;
    }
}
