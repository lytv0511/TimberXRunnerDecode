package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "StudioTestAutoRed", group = "Autonomous")
public class StudioTestAutoRed extends LinearOpMode {

    // --- Drive ---
    private MecanumDrive drive;

    // --- Intake / launcher hardware (mirrors StudioRunnerAutoRed) ---
    private DcMotorEx launcherFlywheel;
    private DcMotor launcherElevator;
    private DcMotor sorter;
    private DcMotor intake;

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

        // --- Autonomous movement ---
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        // move backward 51 inches (negative X)
                        .lineToX(-51)
                        // clockwise turn is negative heading
                        .turn(Math.toRadians(-45))
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
}
