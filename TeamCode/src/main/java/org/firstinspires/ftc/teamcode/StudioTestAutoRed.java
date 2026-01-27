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

        // --- Drive backward first ---
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(-51)
                        .build()
        );

        // --- Launch sequence before turning ---
        defaultLaunchSequence();

        // --- Turn clockwise 60 degrees ---
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turn(Math.toRadians(-75))
                        .build()
        );

        // --- Strafe right 5 inches ---
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(new Vector2d(
                                drive.localizer.getPose().position.x,
                                drive.localizer.getPose().position.y + 5
                        ))
                        .build()
        );

        // --- Drive forward 10 inches ---
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(drive.localizer.getPose().position.x + 10)
                        .build()
        );

        // --- Telemetry after motion ---
        Pose2d pose = drive.localizer.getPose();
        telemetry.addLine("Motion complete");
        telemetry.addData("X (in)", "%.1f", pose.position.x);
        telemetry.addData("Y (in)", "%.1f", pose.position.y);
        telemetry.update();

        // --- Intake sequence ---
        defaultIntakeSequence();

        telemetry.addLine("Auto complete");
        telemetry.addData("Balls Sorted", ballCount);
        telemetry.addData("Pattern", storePatternBuilder.toString());
        telemetry.update();

        sleep(2000);
    }

    // === Simplified intake sequence copied from StudioRunnerAutoRed ===
    private void defaultIntakeSequence() {
        if (launcherSequenceBusy) return;
        launcherSequenceBusy = true;

        final double NUDGE_DISTANCE = 3.0;
        final int MAX_NUDGES = 5;
        final double DETECTION_TIMEOUT = 2.0;

        intake.setPower(-1);
        launcherElevator.setPower(0.2);
        sensorActive = true;

        int nudges = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && nudges < MAX_NUDGES && ballCount < 3) {

            if (timer.seconds() > DETECTION_TIMEOUT) {
                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .lineToX(drive.localizer.getPose().position.x + NUDGE_DISTANCE)
                                .build()
                );
                nudges++;
                timer.reset();
            }

            idle();
        }

        intake.setPower(0);
        launcherElevator.setPower(0);
        sensorActive = false;

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
}
