package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "StudioTestTeleop", group = "TeleOp")
public class StudioTestTeleop extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private CRServo leftIntake;
    private CRServo rightIntake;

    private DcMotor intakeWheel;
    private DcMotor rubber;

    @Override
    public void runOpMode() {

        // Hardware configuration
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");

        intakeWheel = hardwareMap.get(DcMotor.class, "intakeWheel");
        rubber = hardwareMap.get(DcMotor.class, "rubber");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Joystick inputs
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x;  // Strafing
            double rx = gamepad1.right_stick_x; // Rotation

            // Mecanum drive calculations
            double denominator = Math.max(
                    Math.abs(y) + Math.abs(x) + Math.abs(rx), 1
            );

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Apply motor powers
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // GAMEPAD OPTION BUTTONS
            if (gamepad1.right_trigger == 1.0) {
                // Spin all intakes
                leftIntake.setPower(1.0);
                rightIntake.setPower(-1.0);
                intakeWheel.setPower(1.0);
                rubber.setPower(-1.0);

            } else {
                // Stop all intakes
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                intakeWheel.setPower(0);
                rubber.setPower(0);

            }

            telemetry.addData("Forward", y);
            telemetry.addData("Strafe", x);
            telemetry.addData("Rotation", rx);
            telemetry.update();
        }
    }
}
