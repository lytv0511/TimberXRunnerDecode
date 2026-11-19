package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.vision.PollenIntakeAssist;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

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
 *   A (held)      - auto-intake assist: webcam sees pollen/nectar, robot drives
 *                   to it and runs leftIntake, rightIntake, intakeWheel.
 *                   Assist owns drive + intake while held; other mechanisms
 *                   keep their manual controls.
 *
 * Vision hardware: "webcam" (Logitech, USB 2.0) for color; the "limelight"
 * (USB 3.0) is AprilTag-only and is not used in this TeleOp.
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

    private PollenVision vision;
    private PollenIntakeAssist assist;
    private boolean visionAvailable = false;

    @Override
    public void runOpMode() {
        drive = new BasicMecanumDrive(hardwareMap);

        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        intakeWheel = hardwareMap.get(DcMotor.class, "intakeWheel");
        flywheel = hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);
        rubber = hardwareMap.get(DcMotor.class, "rubber");
        two = hardwareMap.get(DcMotor.class, TWO_NAME);

        // Webcam is optional at runtime: if it's not configured, manual controls
        // still work and the A-button assist is simply disabled.
        try {
            vision = new PollenVision();
            vision.init(hardwareMap);
            assist = new PollenIntakeAssist(drive, leftIntake, rightIntake, intakeWheel, vision);
            visionAvailable = true;
        } catch (RuntimeException e) {
            vision = null;
            assist = null;
            visionAvailable = false;
        }

        telemetry.addLine("StudioRunnerTestTeleop ready");
        telemetry.addData("New devices", "flywheel=%s, Two=%s", FLYWHEEL_NAME, TWO_NAME);
        telemetry.addData("Controls", "RT=intake, LT=flywheel, LB=rubber, RB=Two(rev), A=assist");
        telemetry.addData("Webcam assist", visionAvailable ? "ready (\"webcam\")" : "UNAVAILABLE");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double forward = 0;
            double strafe = 0;
            double turn = 0;
            boolean intakeOn = false;
            String assistStatus = visionAvailable ? "off" : "no webcam";

            boolean assistHeld = visionAvailable && gamepad1.a;
            if (assistHeld) {
                // Assist owns drive + intake for this loop.
                assistStatus = assist.update(true);
            } else {
                if (visionAvailable) assistStatus = assist.update(false);

                // --- Drive: two sticks ---
                forward = -gamepad1.left_stick_y * DRIVE_SPEED;
                strafe = gamepad1.left_stick_x * DRIVE_SPEED;
                turn = gamepad1.right_stick_x * DRIVE_SPEED;
                drive.setDrivePowers(forward, strafe, turn);

                // --- Right trigger: intake group ---
                intakeOn = gamepad1.right_trigger > TRIGGER_THRESHOLD;
                leftIntake.setPower(intakeOn ? LEFT_INTAKE_POWER : 0);
                rightIntake.setPower(intakeOn ? RIGHT_INTAKE_POWER : 0);
                intakeWheel.setPower(intakeOn ? INTAKE_WHEEL_POWER : 0);
            }

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
            telemetry.addData("Intake (RT)", intakeOn ? "ON" : "off");
            telemetry.addData("A assist", assistStatus);
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "off");
            telemetry.addData("Rubber", rubberOn ? "ON" : "off");
            telemetry.addData("Two (rev)", twoOn ? "ON" : "off");
            telemetry.update();

            idle();
        }

        stopAll();
    }

    private void stopAll() {
        if (assist != null) assist.stopAll();
        else drive.stop();
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        intakeWheel.setPower(0);
        flywheel.setPower(0);
        rubber.setPower(0);
        two.setPower(0);
        if (vision != null) vision.close();
    }
}
