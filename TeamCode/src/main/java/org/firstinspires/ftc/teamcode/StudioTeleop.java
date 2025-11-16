package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="StudioTeleop", group="TeleOp")
public class StudioTeleop extends OpMode {

    private DcMotor launcherRight;
    private DcMotor launcherLeft;
    private DcMotor sorter;
    private CRServo intakeServo;
    private MecanumDrive drive;
    private StudioAprilTag aprilTag;
    private double launchTargetTicks = 0;

    @Override
    public void init() {
        // Map motors from the configuration
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // Adjust direction if needed
        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        sorter.setTargetPosition(1);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive = new MecanumDrive(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));

        aprilTag = new StudioAprilTag();
        aprilTag.init(hardwareMap, "Webcam 1");

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

        aprilTag.updatePose();
        int tagId = aprilTag.getTagId();
        org.firstinspires.ftc.robotcore.external.navigation.Position pos = aprilTag.getRobotPosition();

        double distance = 0;
        if (tagId == 24 && pos != null) {
            double x = pos.x;
            double z = pos.z;
            distance = Math.sqrt(x * x + z * z);
            telemetry.addData("Tag X (in)", x);
            telemetry.addData("Tag Y (in)", pos.y);
            telemetry.addData("Tag Z (in)", z);
        }

        // Convert distance to ticks (example linear function, adjust later)
        launchTargetTicks = distance / 100;

        // Right trigger on gamepad1 goes from 0.0 to 1.0
        double triggerPower = 0;
        double launcherTrigger = 0;
        boolean intakeOn = false;

//        if (gamepad1.left_bumper && gamepad1.right_bumper) {
//            launcherTrigger = gamepad1.right_trigger - gamepad1.left_trigger;
//            triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
//        } else if (gamepad1.right_bumper) {
//            triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
//        } else if (gamepad1.left_bumper) {
//            launcherTrigger = gamepad1.right_trigger - gamepad1.left_trigger;
//        }

        if (gamepad1.right_bumper) {
            intakeOn = true;
        } else if (gamepad1.left_bumper) {
            intakeOn = false;
        }

        launcherTrigger = gamepad1.right_trigger;
        triggerPower = gamepad1.left_trigger;

        if (intakeOn) {
            intakeServo.setPower(1);
        } else if (!intakeOn) {
            intakeServo.setPower(0);
        }

        if (gamepad1.a) {
            triggerPower = 0.8;
        }

        if (gamepad1.b) {
            triggerPower = 1.0;
        }

        if (gamepad1.x && tagId == 24 && pos != null) {
            launcherRight.setPower(launchTargetTicks);
        }

        if (gamepad1.y) {
            triggerPower = 0.75;
        }

        if (gamepad1.dpad_up) {
            sorter.setTargetPosition(1);
        } else if (gamepad1.dpad_right) {
            sorter.setTargetPosition(0);
        } else if (gamepad1.dpad_left) {
            sorter.setTargetPosition(-1);
        }

        // Set both launchers to that power
        launcherRight.setPower(triggerPower);
        launcherLeft.setPower(-launcherTrigger);

        telemetry.addData("Trigger", triggerPower);
        telemetry.addData("Launcher1 Power", launcherRight.getPower());
        telemetry.addData("Launcher2 Power", launcherLeft.getPower());
        telemetry.addData("TagID", tagId);
        telemetry.addData("Distance (in)", distance);
        telemetry.addData("Launch Target Ticks", launchTargetTicks);
    }

    @Override
    public void stop() {
        // Stop motors when TeleOp ends
        launcherRight.setPower(0.0);
        launcherLeft.setPower(0.0);
    }
}
