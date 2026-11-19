package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * StudioRunner test teleop.
 *
 * Controls (gamepad1):
 *   Left stick Y  - forward / backward
 *   Left stick X  - strafe left / right
 *   Right stick X - rotate
 *   Right trigger - intake group: leftIntake, rightIntake, intakeWheel
 *   Left trigger  - flywheel
 *   Left bumper   - rubber
 *   Right bumper  - "Two" motor (reverse direction)
 *
 * All four mechanisms are independent and can be held at the same time.
 */
@Config
@TeleOp(name = "StudioRunnerTestTeleop", group = "TeleOp")
public class StudioRunnerTestTeleop extends LinearOpMode {

    // Device names in the Robot Controller config. Change here if yours differ.
    private static final String FLYWHEEL_NAME = "FlyWheel";
    private static final String TWO_NAME = "Two";

    // Dashboard tunables
    public static double DRIVE_SPEED = 1.0;
    public static double TRIGGER_THRESHOLD = 0.1;
    public static double LEFT_INTAKE_POWER = 1.0;
    public static double RIGHT_INTAKE_POWER = -1.0;
    public static double INTAKE_WHEEL_POWER = 1.0;
    public static double FLYWHEEL_POWER = 1.0;
    public static double RUBBER_POWER = -1.0;
    public static double TWO_POWER = -1.0; // reversed from the original direction

    private BasicMecanumDrive drive;
    private CRServo leftIntake;
    private CRServo rightIntake;
    private DcMotor intakeWheel;
    private DcMotor flywheel;
    private DcMotor rubber;
    private DcMotor two;

    @Override
    public void runOpMode() {
        drive = new BasicMecanumDrive(hardwareMap);

        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        intakeWheel = hardwareMap.get(DcMotor.class, "intakeWheel");
        flywheel = hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);
        rubber = hardwareMap.get(DcMotor.class, "rubber");
        two = hardwareMap.get(DcMotor.class, TWO_NAME);

        telemetry.addLine("StudioRunnerTestTeleop ready");
        telemetry.addData("New devices", "flywheel=%s, Two=%s", FLYWHEEL_NAME, TWO_NAME);
        telemetry.addData("Controls", "RT=intake, LT=flywheel, LB=rubber, RB=Two(rev)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Drive: two sticks ---
            double forward = -gamepad1.left_stick_y * DRIVE_SPEED;
            double strafe = gamepad1.left_stick_x * DRIVE_SPEED;
            double turn = gamepad1.right_stick_x * DRIVE_SPEED;
            drive.setDrivePowers(forward, strafe, turn);

            // --- Right trigger: intake group ---
            boolean intakeOn = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            leftIntake.setPower(intakeOn ? LEFT_INTAKE_POWER : 0);
            rightIntake.setPower(intakeOn ? RIGHT_INTAKE_POWER : 0);
            intakeWheel.setPower(intakeOn ? INTAKE_WHEEL_POWER : 0);

            // --- Left trigger: flywheel ---
            boolean flywheelOn = gamepad1.left_trigger > TRIGGER_THRESHOLD;
            flywheel.setPower(flywheelOn ? FLYWHEEL_POWER : 0);

            // --- Left bumper: rubber ---
            boolean rubberOn = gamepad1.left_bumper;
            rubber.setPower(rubberOn ? RUBBER_POWER : 0);

            // --- Right bumper: Two motor (reverse direction) ---
            boolean twoOn = gamepad1.right_bumper;
            two.setPower(twoOn ? TWO_POWER : 0);

            telemetry.addData("Drive F/S/R", "%.2f / %.2f / %.2f", forward, strafe, turn);
            telemetry.addData("Intake", intakeOn ? "ON" : "off");
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "off");
            telemetry.addData("Rubber", rubberOn ? "ON" : "off");
            telemetry.addData("Two (rev)", twoOn ? "ON" : "off");
            telemetry.update();

            idle();
        }

        stopAll();
    }

    private void stopAll() {
        drive.stop();
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        intakeWheel.setPower(0);
        flywheel.setPower(0);
        rubber.setPower(0);
        two.setPower(0);
    }
}
