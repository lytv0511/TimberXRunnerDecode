package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RunMotors", group="TeleOp")
public class RunMotors extends OpMode {

    private DcMotor launcherRight;
    private DcMotor launcherLeft;
    private MecanumDrive drive;

    @Override
    public void init() {
        // Map motors from the configuration
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");

        // Adjust direction if needed
        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        drive = new MecanumDrive(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
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

        // Right trigger on gamepad1 goes from 0.0 to 1.0
        double triggerPower = 0;
        double launcherTrigger = 0;

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            launcherTrigger = gamepad1.right_trigger - gamepad1.left_trigger;
            triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
        } else if (gamepad1.right_bumper) {
            triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
        } else if (gamepad1.left_bumper) {
            launcherTrigger = gamepad1.right_trigger - gamepad1.left_trigger;
        }

        if (gamepad1.a) {
            triggerPower = 0.8;
        }

        if (gamepad1.b) {
            triggerPower = 1.0;
        }

        if (gamepad1.x) {
            triggerPower = 0.7;
        }

        if (gamepad1.y) {
            triggerPower = 0.75;
        }

        // Set both launchers to that power
        launcherRight.setPower(triggerPower);
        launcherLeft.setPower(-launcherTrigger);

        telemetry.addData("Trigger", triggerPower);
        telemetry.addData("Launcher1 Power", launcherRight.getPower());
        telemetry.addData("Launcher2 Power", launcherLeft.getPower());
    }

    @Override
    public void stop() {
        // Stop motors when TeleOp ends
        launcherRight.setPower(0.0);
        launcherLeft.setPower(0.0);
    }
}
