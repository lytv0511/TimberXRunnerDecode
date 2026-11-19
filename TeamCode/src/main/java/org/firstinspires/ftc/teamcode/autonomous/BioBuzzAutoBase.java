package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BasicMecanumDrive;

/**
 * Base for all BIOBUZZ autos on the Basic Mecanum config:
 *   leftFront/rightFront/leftBack/rightBack (DcMotor),
 *   leftIntake/rightIntake (CRServo), intakeWheel, rubber.
 *
 * Open-loop and timed: no IMU, encoders, camera, or RoadRunner trajectories.
 * The vision/search classes in autonomous.search are kept for a config that
 * has a camera, but they are not used here.
 */
@Config
public abstract class BioBuzzAutoBase extends LinearOpMode {

    protected BasicMecanumDrive drive;
    protected CRServo leftIntake;
    protected CRServo rightIntake;
    protected DcMotor intakeWheel;
    protected DcMotor rubber;
    protected PollenWorkflow pollen;

    /** Red = +1, Blue = -1. Mirrors strafe and turn commands. */
    protected abstract double getAllianceSign();

    /** Implement the auto sequence here — keep it linear/readable. */
    protected abstract void runAuto();

    protected int ballCount = 0;

    @Override
    public final void runOpMode() {
        initHardware();

        telemetry.addLine("BioBuzz auto ready (Basic Mecanum config)");
        telemetry.addData("Alliance sign", getAllianceSign());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runAuto();

        telemetry.addLine("Auto complete");
        telemetry.addData("Balls handled", ballCount);
        telemetry.update();
        stopHardware();
    }

    protected void initHardware() {
        drive = new BasicMecanumDrive(hardwareMap);
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        intakeWheel = hardwareMap.get(DcMotor.class, "intakeWheel");
        rubber = hardwareMap.get(DcMotor.class, "rubber");

        pollen = new PollenWorkflow(this);
        allOff();
    }

    protected void stopHardware() {
        allOff();
        drive.stop();
    }

    // ----- Open-loop drive helpers -----

    protected void driveFor(double seconds, double forward, double strafe, double turn) {
        if (seconds <= 0) return;
        ElapsedTime t = new ElapsedTime();
        drive.setDrivePowers(forward, strafe, turn);
        while (opModeIsActive() && t.seconds() < seconds) idle();
        drive.stop();
    }

    protected void driveForward(double seconds, double power) {
        driveFor(seconds, power, 0, 0);
    }

    protected void driveForward(double seconds) {
        driveForward(seconds, BioBuzzConstants.DRIVE_FORWARD_POWER);
    }

    protected void backOffWall(double seconds) {
        driveForward(seconds, -BioBuzzConstants.DRIVE_FORWARD_POWER);
    }

    protected void strafeBy(double seconds, double power) {
        driveFor(seconds, 0, power * getAllianceSign(), 0);
    }

    protected void turnBy(double seconds, double power) {
        driveFor(seconds, 0, 0, power * getAllianceSign());
    }

    protected void waitSeconds(double seconds) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) idle();
    }

    // ----- Mechanism helpers -----

    /** Intake rollers/wheel only; rubber (launcher) stays off. */
    protected void intakeOn() {
        leftIntake.setPower(BioBuzzConstants.LEFT_INTAKE_POWER);
        rightIntake.setPower(BioBuzzConstants.RIGHT_INTAKE_POWER);
        intakeWheel.setPower(BioBuzzConstants.INTAKE_WHEEL_POWER);
        rubber.setPower(0);
    }

    protected void intakeOff() {
        allOff();
    }

    /** Spin the rubber launcher, then feed N balls with the intake mechanisms. */
    protected void shoot(int balls) {
        if (!opModeIsActive() || balls <= 0) return;

        rubber.setPower(BioBuzzConstants.RUBBER_POWER);
        waitSeconds(BioBuzzConstants.SHOOT_SPINUP_S);

        for (int i = 0; i < balls && opModeIsActive(); i++) {
            leftIntake.setPower(BioBuzzConstants.LEFT_INTAKE_POWER);
            rightIntake.setPower(BioBuzzConstants.RIGHT_INTAKE_POWER);
            intakeWheel.setPower(BioBuzzConstants.INTAKE_WHEEL_POWER);
            waitSeconds(BioBuzzConstants.SHOOT_FEED_S);

            leftIntake.setPower(0);
            rightIntake.setPower(0);
            intakeWheel.setPower(0);
            waitSeconds(BioBuzzConstants.SHOOT_GAP_S);

            ballCount = Math.max(0, ballCount - 1);
        }

        rubber.setPower(0);
    }

    protected void allOff() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        intakeWheel.setPower(0);
        rubber.setPower(0);
    }
}
