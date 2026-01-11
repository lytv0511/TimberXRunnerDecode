package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Autonomous(name="StudioRunnerAutoRed", group = "Autonomous")
public class StudioRunnerAutoRed extends LinearOpMode {
    private StudioAprilTag studioAprilTag;

    // Hardware declarations
    private com.qualcomm.robotcore.hardware.DcMotorEx launcherFlywheel;
    private com.qualcomm.robotcore.hardware.DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private com.qualcomm.robotcore.hardware.DcMotor launcherElevator;
    private com.qualcomm.robotcore.hardware.DcMotor sorter;
    private com.qualcomm.robotcore.hardware.CRServo intakeServo;
    private MecanumDrive drive;

    // Shared intake/sorter state copied from StudioTeleop
    private boolean sensorActive = false;
    private int ballCount = 0;
    private StringBuilder storePatternBuilder = new StringBuilder();
    private int lastSensorColor = 0;
    private String detectedPattern = "Unknown";

    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;

    private double ticksPerRevolution = 537.7;
    private double augPos1, augPos2, augPos3;

    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;

    double targetVelocity = 53770;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hardware initialization
        launcherFlywheel = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "launcherFlywheel");
        launcherElevator = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "launcherElevator");
        sorter = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "sorter");
        intakeServo = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "intakeServo");
        colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorSensor");

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE);
        launcherElevator.setDirection(DcMotorEx.Direction.REVERSE);

        augPos1 = ticksPerRevolution / 2;
        augPos2 = (ticksPerRevolution * 5) / 6;
        augPos3 = ticksPerRevolution / 6;

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        launcherFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherFlywheel.setPIDFCoefficients(
                launcherFlywheel.getMode(), new PIDFCoefficients(10, 0, 0, 15));

        launcherFlywheel.setVelocity(targetVelocity);


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
                        .strafeTo(new Vector2d(0, -2.5))
                        .setTangent(Math.toRadians(180)) // now we want to go backwards toward -X
                        .lineToX(-52) // was -40 for blue
                        .turn(Math.toRadians(10))
                        .build()
        );

        defaultLaunchSequence();

//        Pose2d currentPos = drive.localizer.getPose();
//        Actions.runBlocking(
//                drive.actionBuilder(currentPos)
//                        .turn(Math.toRadians(-105))
//                        .lineToY(-60)
//                        .turn(Math.toRadians(105))
//                        .build()
//        );

        Pose2d currentPos = drive.localizer.getPose();

        Actions.runBlocking(
                drive.actionBuilder(currentPos)
                        .turn(Math.toRadians(-15))
                        .strafeTo( new Vector2d(currentPos.position.x + 42, currentPos.position.y - 65))
                        //.turn(Math.toRadians(-10))
                        .build()
        );

//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(0, 0, 0))
//                        .turn(Math.toRadians(-40))
//                        .lineToY(62)
//                        .turn(Math.toRadians(40))
//                        .build()
//        );

        defaultIntakeSequence();

        tagId = detectTagID();
        telemetry.addData("Detected Tag ID after scanning", tagId);

        String tagName = studioAprilTag.getTagName();
        if (tagName != null) {
            telemetry.addData("Tag Name", tagName);
        }

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
        telemetry.update();

        Action trajectoryActionChosen;
        Pose2d currentPose = drive.localizer.getPose();
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
        sleep(5000);
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

//    private void defaultIntakeSequence() {
//        // If another sequence is running, ignore
//        if (launcherSequenceBusy) return;
//        launcherSequenceBusy = true;
//
//        final double INTAKE_FORWARD_DISTANCE = 3.0; // inches per nudge
//        final double INITIAL_INTAKE_FORWARD_DISTANCE = 0.0; // inches per nudge
//        final int MAX_FORWARD_ATTEMPTS = 4;
//        final double DETECTION_TIMEOUT = 3.0; // seconds
//
//        // prepare
//        launcherElevator.setPower(0.2);
//        launcherFlywheel.setPower(0);
//        sensorActive = true;
//        intakeServo.setPower(-1); // start intake
//
//        boolean initialAState = gamepad1.a; // capture current A state so a subsequent press can cancel
//        boolean canceled = false;
//
//        // Ensure sorter starts at pos1
////        sorter.setTargetPosition(0);
////        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        sorter.setPower(0.3);
////        while (sorter.isBusy() && opModeIsActive()) { idle(); }
////        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        moveSorterToPos3();
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.localizer.getPose())
//                        .lineToX(drive.localizer.getPose().position.x + INITIAL_INTAKE_FORWARD_DISTANCE)
//                        .build()
//        );
//
//        // main loop: collect up to 3 balls unless interrupted
//        int lastHandledBallCount = 0;
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        int forwardAttempts = 0;
//        ElapsedTime detectionTimer = new ElapsedTime();
//        detectionTimer.reset();
//
//        while (opModeIsActive() && ballCount < 4 && !canceled) {
//
//            // auto-cancel after 5 seconds
//            if (timer.seconds() >= 10.0) {
//                canceled = true;
//                break;
//            }
//
//            // detect manual cancel (X or second A press)
//            if (gamepad1.x) {
//                canceled = true;
//                break;
//            }
//            boolean curA = gamepad1.a;
//            if (curA && !initialAState) {
//                canceled = true;
//                break;
//            }
//
//            // read sensor (will update ballCount/storePatternBuilder)
//            readColorSensor();
//
//            // ---- Forward nudge logic if no ball detected ----
//            if (ballCount == lastHandledBallCount) {
//                // If we have waited too long with no new ball, move forward again
//                if (detectionTimer.seconds() >= DETECTION_TIMEOUT &&
//                        forwardAttempts < MAX_FORWARD_ATTEMPTS) {
//
//                    Actions.runBlocking(
//                            drive.actionBuilder(drive.localizer.getPose())
//                                    .lineToX(drive.localizer.getPose().position.x + INTAKE_FORWARD_DISTANCE)
//                                    .build()
//                    );
//
//                    forwardAttempts++;
//                    detectionTimer.reset();
//                }
//
//                // Stop trying after max attempts
//                if (forwardAttempts >= MAX_FORWARD_ATTEMPTS &&
//                        detectionTimer.seconds() >= DETECTION_TIMEOUT) {
//                    canceled = true;
//                }
//            }
//
//            // if a new ball was detected, move sorter to next pos immediately
//            if (ballCount > lastHandledBallCount) {
//                forwardAttempts = 0;
//                detectionTimer.reset();
//                if (ballCount == 1) {
//                    moveSorterToPos3();
//                    while (sorter.isBusy() && opModeIsActive()) { idle(); }
//                } else if (ballCount == 2) {
//                    moveSorterToPos1();
//                    while (sorter.isBusy() && opModeIsActive()) { idle(); }
//                } else if (ballCount == 3) {
//                    moveSorterToPos2();
//                } else if (ballCount >= 4) {
//                    break;
//                }
//                lastHandledBallCount += 1;
//            }
//
//            idle();
//        }
//
//        // stop intake & sensor
//        sensorActive = false;
//        intakeServo.setPower(0);
//        launcherElevator.setPower(0);
//        launcherFlywheel.setPower(0);
//
//        // If canceled by X (manual reset), clear pattern immediately
//        if (gamepad1.x || canceled) {
//            ballCount = 0;
//            storePatternBuilder.setLength(0);
//            lastSensorColor = 0;
//        }
//
//        // ensure sorter returns to pos1 (so launcher can read balls)
    ////        sorter.setTargetPosition(0);
    ////        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ////        sorter.setPower(0.3);
    ////        while (sorter.isBusy() && opModeIsActive()) { idle(); }
    ////        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ////        sorter.setPower(0);
    ////        moveSorterToPos3();
    ////        intakeServo.setPower(-1);
    ////        sleep(3000);
//        intakeServo.setPower(0);
//
//        telemetry.addData("Stored Pattern", storePatternBuilder.toString());
//        telemetry.addData("Balls Loaded", ballCount);
//        telemetry.update();
//
//        launcherSequenceBusy = false;
//    }




    private void defaultIntakeSequence() {
        // 1. Prevent overlapping sequences
        if (launcherSequenceBusy) return;
        launcherSequenceBusy = true;

        // --- Configuration ---
        final double NUDGE_DISTANCE = 3.0; // Inches to move forward if no ball is seen
        final int MAX_NUDGES_PER_BALL = 5; // Max attempts to find the next ball
        final double DETECTION_TIMEOUT = 2.0; // Seconds to wait before deciding to nudge
        final int TARGET_BALLS = 4;

        // --- Hardware Setup ---
        launcherElevator.setPower(0.2);
        intakeServo.setPower(-1); // Start spinning the rubber intake
        sensorActive = true;

        int lastHandledBallCount = 0;
        int currentBallNudges = 0;

        ElapsedTime sequenceTimer = new ElapsedTime();
        ElapsedTime nudgeTimer = new ElapsedTime();
        sequenceTimer.reset();
        nudgeTimer.reset();

        // --- Main Intake Loop ---
        // Condition: Loop continues until the 3rd ball has been FULLY processed/sorted
        while (opModeIsActive() && lastHandledBallCount < TARGET_BALLS) {

            // Manual cancel or safety timeout (15s)
            if (gamepad1.x || sequenceTimer.seconds() > 15.0) {
                break;
            }

            // Always update sensor state
            readColorSensor();

            // LOGIC A: We are currently searching for a ball (Sensor hasn't seen a new one yet)
            if (ballCount == lastHandledBallCount) {
                if (nudgeTimer.seconds() >= DETECTION_TIMEOUT) {
                    if (currentBallNudges < MAX_NUDGES_PER_BALL) {
                        // Drive forward slightly to reach the ball
                        Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                        .lineToX(drive.localizer.getPose().position.x + NUDGE_DISTANCE)
                                        .build()
                        );
                        currentBallNudges++;
                        nudgeTimer.reset(); // Wait at new position for detection
                    } else {
                        // Stop the loop if we've nudged too many times without finding a ball
                        break;
                    }
                }
            }

            // LOGIC B: A new ball was just detected by the color sensor
            else if (ballCount > lastHandledBallCount) {
                // New ball detected! Immediately handle sorter movement
                if (ballCount == 1) {
                    moveSorterToPos3();
                } else if (ballCount == 2) {
                    moveSorterToPos1();
                } else if (ballCount == 3) {
                    moveSorterToPos2();
                }

                // Wait for the sorter to finish rotating before doing anything else
                while (sorter.isBusy() && opModeIsActive()) {
                    idle();
                }

                // Successfully processed this ball
                lastHandledBallCount = ballCount;

                // Reset "Search" variables for the NEXT ball
                currentBallNudges = 0;
                nudgeTimer.reset();
            }

            idle();
        }

        // --- Cleanup and Shutdown ---
        sensorActive = false;
        intakeServo.setPower(0);
        launcherElevator.setPower(0);
        launcherFlywheel.setPower(0);

        // If we canceled or timed out, reset variables for next time
        if (gamepad1.x) {
            ballCount = 0;
            storePatternBuilder.setLength(0);
        }

        telemetry.addData("Sequence Status", "Complete");
        telemetry.addData("Total Balls Sorted", lastHandledBallCount);
        telemetry.update();

        launcherSequenceBusy = false;
    }
    private void defaultLaunchSequence() {
        launcherSequenceBusy = true;
        boolean canceled = false;
        final double MOVEMENT_DISTANCE = 2.5; // Distance to move in inches per D-pad press
        final double DRIVE_POWER = 0.4;
        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Configure flywheel PIDF with velocity control ---
        final double LAUNCHER_TARGET_VELOCITY = 1780; // ticks/sec

        launcherFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10)
        );

        launcherFlywheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
        sleep(1000);

        // --- Ball positions ---
        double[] augPositions = {augPos3, augPos1, augPos2};

        for (double augPos : augPositions) {
            // Wait for flywheel to reach near target speed
            ElapsedTime spinTimer = new ElapsedTime();
            spinTimer.reset();
            while (opModeIsActive() &&
                    Math.abs(launcherFlywheel.getVelocity() - LAUNCHER_TARGET_VELOCITY) > 50) // was 1780
            {
                if (gamepad1.x) {
                    canceled = true;
                    break;
                }

                boolean currentDpadUp = gamepad1.dpad_up;
                boolean currentDpadDown = gamepad1.dpad_down;

                if (currentDpadUp && !lastDpadUpState) {
                    Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(MOVEMENT_DISTANCE)
                            .build());
                    sleep(300); // Temporary placeholder to simulate blocking movement
                } else if (currentDpadDown && !lastDpadDownState) {
                    Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(-MOVEMENT_DISTANCE)
                            .build());
                    sleep(300); // Temporary placeholder to simulate blocking movement
                }

                lastDpadUpState = currentDpadUp;
                lastDpadDownState = currentDpadDown;

                idle();
            }
            if (canceled) break;

            // Move sorter to the ball
            sorter.setTargetPosition((int) augPos);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.3);

            // Wait for sorter to move
            while (sorter.isBusy() && opModeIsActive()) {
                if (gamepad1.x) {
                    canceled = true;
                    break;
                }
                idle();
            }
            if (canceled) break;

            // Wait until the sorter is within a small tolerance of the target position
            ElapsedTime alignmentTimer = new ElapsedTime();
            alignmentTimer.reset();
            int tolerance = 10;

            while (opModeIsActive() &&
                    Math.abs(sorter.getCurrentPosition() - (int)augPos) > tolerance &&
                    alignmentTimer.seconds() < 0.5) {
                if (gamepad1.x) {
                    canceled = true;
                    break;
                }
                idle();
            }
            if (canceled) break;

            // Ensure motor is stopped after alignment for no drift
            sorter.setPower(0);

            // Feed ball using elevator
            launcherElevator.setPower(-1.0);
            ElapsedTime feedTimer = new ElapsedTime();
            feedTimer.reset();

            while (feedTimer.seconds() < 0.7 && opModeIsActive()) {
                if (gamepad1.x) {
                    canceled = true;
                    break;
                }
                idle();
            }
            launcherElevator.setPower(0);
            if (canceled) break;
        }

        if (canceled) {
            launcherFlywheel.setPower(0);
            launcherElevator.setPower(0);

            sorter.setTargetPosition(0); // pos1
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.3);
            while (sorter.isBusy() && opModeIsActive()) { idle(); }

            sorter.setPower(0);
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            launcherSequenceBusy = false;
            return;
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

    private void moveSorterToPos1() {
        double pos2Encoder = 0; // 120°
        sorter.setTargetPosition((int) pos2Encoder);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
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