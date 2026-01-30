package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "StudioTestAutoRed", group = "Autonomous")
public class StudioTestAutoRed extends LinearOpMode {

    // --- Drive ---
    private MecanumDrive drive;
    private int lastSensorColor = 0;

    // --- Intake / launcher hardware (mirrors StudioRunnerAutoRed) ---
    private DcMotorEx launcherFlywheel;
    private DcMotor launcherElevator;
    private DcMotor sorter;
    private DcMotor intake;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;

    // --- Shared intake state ---
    private boolean sensorActive = false;
    private int ballCount = 0;
    private StringBuilder storePatternBuilder = new StringBuilder();

    // --- Constants copied from StudioRunnerAutoRed ---
    private double ticksPerRevolution = 537.7;

    double pos2 = ticksPerRevolution / 3; // 120°
    double pos3 = (ticksPerRevolution * 2) / 3; // 240°
    double pos1 = 0; // 360°
    private double augPos1, augPos2, augPos3;

    private boolean launcherSequenceBusy = false;

    @Override
    public void runOpMode() {

        // --- Init drive ---
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // --- Init hardware ---
        launcherFlywheel = hardwareMap.get(DcMotorEx.class, "launcherFlywheel");
        launcherElevator = hardwareMap.get(DcMotor.class, "launcherElevator");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        launcherElevator.setDirection(DcMotor.Direction.REVERSE);
        sorter.setDirection(DcMotor.Direction.REVERSE);

        augPos1 = ticksPerRevolution / 2;
        augPos2 = (ticksPerRevolution * 5) / 6;
        augPos3 = ticksPerRevolution / 6;

        telemetry.addLine("StudioTestAutoRed ready");
        telemetry.addData("Start Pose", "(0, 0, 0)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        launcherFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10)
        );
        launcherFlywheel.setVelocity(1680);

        // --- Drive backward first ---
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(-51)
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(10))
                        .build()
        );

        defaultLaunchSequence();

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(-85))
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(new Vector2d(
                                drive.localizer.getPose().position.x - 10,
                                drive.localizer.getPose().position.y
                        ))
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(drive.localizer.getPose().position.x + 7)
                        .build()
        );

        // --- Telemetry after motion ---
        Pose2d pose = drive.localizer.getPose();
        telemetry.addLine("Motion complete");
        telemetry.addData("X (in)", "%.1f", pose.position.x);
        telemetry.addData("Y (in)", "%.1f", pose.position.y);
        telemetry.update();

        defaultIntakeSequence();

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(drive.localizer.getPose().position.x - 11.5)
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(new Vector2d(
                                drive.localizer.getPose().position.x + 10,
                                drive.localizer.getPose().position.y
                        ))
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(85))
                        .build()
        );

        defaultLaunchSequence();
    }

    private void defaultIntakeSequence() {
        if (launcherSequenceBusy) return;
        launcherSequenceBusy = true;

        final double SHORT_NUDGE_DISTANCE = 1.25;
        final double LONG_NUDGE_DISTANCE = 1.5;
        final int MAX_NUDGES = 3;
        final double WAIT_TIME = 1.0;

        intake.setPower(-1);
        launcherElevator.setPower(0.2);

        int nudges = 0;
        ElapsedTime timer = new ElapsedTime();

        // --- Immediate first nudge ---
        rotateSorterToNextSlot();
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(drive.localizer.getPose().position.x + SHORT_NUDGE_DISTANCE)
                        .build()
        );
        nudges++;

        timer.reset();

        // --- Subsequent: wait → nudge → rotate ---
        while (opModeIsActive() && nudges < MAX_NUDGES) {

            if (timer.seconds() >= WAIT_TIME) {

                rotateSorterToNextSlot();

                double nudgeDistance = (nudges < 2) ? SHORT_NUDGE_DISTANCE : LONG_NUDGE_DISTANCE;

                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .lineToX(drive.localizer.getPose().position.x + nudgeDistance)
                                .build()
                );

                nudges++;
                timer.reset();
            }

            idle();
        }

        intake.setPower(0);
        launcherElevator.setPower(0);

        launcherSequenceBusy = false;
    }

    private void rotateSorterToNextSlot() {
        ballCount++;

        double targetPos;
        if (ballCount == 1) targetPos = pos1;
        else if (ballCount == 2) targetPos = pos2;
        else targetPos = pos3;

        sorter.setTargetPosition((int) targetPos);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);

        while (sorter.isBusy() && opModeIsActive()) {
            idle();
        }

        sorter.setPower(0);
    }

//    private void defaultLaunchSequence() {
//        launcherSequenceBusy = true;
//        boolean canceled = false;
//        final double MOVEMENT_DISTANCE = 2.5; // Distance to move in inches per D-pad press
//        final double DRIVE_POWER = 0.4;
//        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // --- Configure flywheel PIDF with velocity control ---
//        final double LAUNCHER_TARGET_VELOCITY = 1680; // ticks/sec
//
//        launcherFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        launcherFlywheel.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(300, 0, 0, 10)
//        );
//
//        launcherFlywheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
//        sleep(200);
//
//        // --- Ball positions ---
//        double[] augPositions = {augPos3, augPos1, augPos2};
//
//        for (double augPos : augPositions) {
//            // Wait for flywheel to reach near target speed
//            ElapsedTime spinTimer = new ElapsedTime();
//            spinTimer.reset();
//            while (opModeIsActive() &&
//                    Math.abs(launcherFlywheel.getVelocity() - LAUNCHER_TARGET_VELOCITY) > 50) // was 1680
//            {
//                if (gamepad1.x) {
//                    canceled = true;
//                    break;
//                }
//
//                boolean currentDpadUp = gamepad1.dpad_up;
//                boolean currentDpadDown = gamepad1.dpad_down;
//
//                if (currentDpadUp && !lastDpadUpState) {
//                    Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
//                            .lineToX(MOVEMENT_DISTANCE)
//                            .build());
//                    sleep(300); // Temporary placeholder to simulate blocking movement
//                } else if (currentDpadDown && !lastDpadDownState) {
//                    Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
//                            .lineToX(-MOVEMENT_DISTANCE)
//                            .build());
//                    sleep(300); // Temporary placeholder to simulate blocking movement
//                }
//
//                lastDpadUpState = currentDpadUp;
//                lastDpadDownState = currentDpadDown;
//
//                idle();
//            }
//            if (canceled) break;
//
//            // Move sorter to the ball
//            sorter.setTargetPosition((int) augPos);
//            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            sorter.setPower(0.3);
//
//            // Wait for sorter to move
//            while (sorter.isBusy() && opModeIsActive()) {
//                if (gamepad1.x) {
//                    canceled = true;
//                    break;
//                }
//                idle();
//            }
//            if (canceled) break;
//
//            // Wait until the sorter is within a small tolerance of the target position
//            ElapsedTime alignmentTimer = new ElapsedTime();
//            alignmentTimer.reset();
//            int tolerance = 10;
//
//            while (opModeIsActive() &&
//                    Math.abs(sorter.getCurrentPosition() - (int)augPos) > tolerance &&
//                    alignmentTimer.seconds() < 0.5) {
//                if (gamepad1.x) {
//                    canceled = true;
//                    break;
//                }
//                idle();
//            }
//            if (canceled) break;
//
//            // Ensure motor is stopped after alignment for no drift
//            sorter.setPower(0);
//
//            // Feed ball using elevator
//            launcherElevator.setPower(-1.0);
//            ElapsedTime feedTimer = new ElapsedTime();
//            feedTimer.reset();
//
//            while (feedTimer.seconds() < 0.7 && opModeIsActive()) {
//                if (gamepad1.x) {
//                    canceled = true;
//                    break;
//                }
//                idle();
//            }
//            launcherElevator.setPower(0);
//            if (canceled) break;
//        }
//
//        if (canceled) {
//            launcherFlywheel.setPower(0);
//            launcherElevator.setPower(0);
//
//            sorter.setTargetPosition(0); // pos1
//            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            sorter.setPower(0.3);
//            while (sorter.isBusy() && opModeIsActive()) { idle(); }
//
//            sorter.setPower(0);
//            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            launcherSequenceBusy = false;
//            return;
//        }
//
//        // Return sorter to position 0
//        sorter.setTargetPosition(0);
//        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sorter.setPower(0.3);
//        while (sorter.isBusy() && opModeIsActive()) { idle(); }
//
//        // Stop all motors safely
//        launcherFlywheel.setPower(0);
//        launcherElevator.setPower(0);
//        sorter.setPower(0);
//
//        launcherSequenceBusy = false;
//
//        // Reset intake counters
//        storePatternBuilder.setLength(0);
//        ballCount = 0;
//        lastSensorColor = 0;
//    }
private void defaultLaunchSequence() {
        launcherSequenceBusy = true;
        boolean canceled = false;
        final double MOVEMENT_DISTANCE = 2.5;

        // --- Configure flywheel ---
        final double LAUNCHER_TARGET_VELOCITY = 1680;

        launcherFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10)
        );

        launcherFlywheel.setVelocity(LAUNCHER_TARGET_VELOCITY);

        // Wait briefly for spin-up before starting logic
        sleep(500);

        double[] augPositions = {augPos1, augPos2, augPos3};
        boolean firstBall = true;

        for (double augPos : augPositions) {

            // --- 1. First Ball: Wait for Flywheel Velocity & Alignment ---
            if (firstBall) {
                while (opModeIsActive() &&
                        Math.abs(launcherFlywheel.getVelocity() - LAUNCHER_TARGET_VELOCITY) > 50)
                {
                    if (gamepad1.x) {
                        canceled = true;
                        break;
                    }

                    // D-Pad alignment logic
                    boolean currentDpadUp = gamepad1.dpad_up;
                    boolean currentDpadDown = gamepad1.dpad_down;

                    if (currentDpadUp && !lastDpadUpState) {
                        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(MOVEMENT_DISTANCE)
                                .build());
                        sleep(100);
                    } else if (currentDpadDown && !lastDpadDownState) {
                        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(-MOVEMENT_DISTANCE)
                                .build());
                        sleep(100);
                    }
                    lastDpadUpState = currentDpadUp;
                    lastDpadDownState = currentDpadDown;

                    idle();
                }
                if (canceled) break;
                firstBall = false;
            }

            // --- 2. Move Sorter (Wait for Completion) ---
            sorter.setTargetPosition((int) augPos);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.3); // Max speed

            // BLOCKING WAIT: Ensure sorter is physically there before feeding
            while (opModeIsActive() && sorter.isBusy()) {
                if (gamepad1.x) {
                    canceled = true;
                    break;
                }
                idle();
            }

            // EXTRA SAFETY: Ensure position is actually within tolerance to prevent jams
            // Sometimes isBusy() finishes slightly early. This ensures alignment.
            while (opModeIsActive() && Math.abs(sorter.getCurrentPosition() - augPos) > 20) {
                if (gamepad1.x) { canceled = true; break; }
                idle();
            }
            if (canceled) break;

            // Stop sorter explicitly to hold position
            sorter.setPower(0);

            // --- 3. Feed Ball (Only runs after Sorter is confirmed aligned) ---
            launcherElevator.setPower(-1.0);
            ElapsedTime feedTimer = new ElapsedTime();
            feedTimer.reset();

            // Short feed time for rapid fire
            while (feedTimer.seconds() < 0.25 && opModeIsActive()) {
                if (gamepad1.x) {
                    canceled = true;
                    break;
                }
                idle();
            }
            launcherElevator.setPower(0);
            if (canceled) break;
        }

        // --- Cleanup ---
        if (canceled) {
            launcherFlywheel.setPower(0);
            launcherElevator.setPower(0);

            // Reset sorter safely
            sorter.setTargetPosition(0);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(1.0);
            while (sorter.isBusy() && opModeIsActive()) { idle(); }
            sorter.setPower(0);

            launcherSequenceBusy = false;
            return;
        }

        // Reset sorter to 0
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }

        launcherFlywheel.setPower(0);
        launcherElevator.setPower(0);
        sorter.setPower(0);

        launcherSequenceBusy = false;
        ballCount = 0;
        lastSensorColor = 0;
    }
}
