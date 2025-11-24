package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="StudioTeleop", group="TeleOp")
public class StudioTeleop extends OpMode {

    private DcMotor launcherFlywheel;
    private DcMotor launcherElevator;
    private DcMotor sorter;
    private CRServo intakeServo;
    private MecanumDrive drive;
    private StudioAprilTag aprilTag;
    private double launchTargetTicks = 0;

    @Override
    public void init() {
        // Map motors from the configuration
        launcherFlywheel = hardwareMap.get(DcMotor.class, "launcherFlywheel");
        launcherElevator = hardwareMap.get(DcMotor.class, "launcherElevator");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // Adjust direction if needed
        launcherFlywheel.setDirection(DcMotor.Direction.FORWARD);
        launcherElevator.setDirection(DcMotor.Direction.REVERSE);
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        // === APRILTAG LOCALIZATION & TELEMETRY ===
        aprilTag.updatePose();

        int tagId = aprilTag.getTagId();
        String tagName = aprilTag.getTagName();
        double yaw = aprilTag.getYawDegrees();
        org.firstinspires.ftc.robotcore.external.navigation.Position pos = aprilTag.getRobotPosition();

        telemetry.addLine("=== AprilTag Detection (TeleOp) ===");
        double distance = 0;
        if (tagId != -1) {
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Tag Name", tagName != null ? tagName : "Unknown");
            if (pos != null) {
                double x = pos.x;
                double yPos = pos.y;
                double z = pos.z;

                telemetry.addData("X (in)", "%.2f", x);
                telemetry.addData("Y (in)", "%.2f", yPos);
                telemetry.addData("Z (in)", "%.2f", z);

                // Calculate distance ignoring yPos
                distance = Math.sqrt(x * x + z * z);
                telemetry.addData("Distance (in)", "%.2f", distance);

                // Convert distance to ticks (adjust as needed)
                launchTargetTicks = distance / 100.0;

                // Fire using proportional power based on distance
                if (gamepad1.x && tagId == 24) {
                    launcherFlywheel.setPower(launchTargetTicks);
                }

                telemetry.addData("Launch Target Ticks", "%.4f", launchTargetTicks);
            } else {
                telemetry.addLine("No valid pose detected.");
            }
            telemetry.addData("Yaw (deg)", "%.2f", yaw);
        } else {
            telemetry.addLine("No AprilTag detected.");
        }

        // Right trigger on gamepad1 goes from 0.0 to 1.0
        double triggerPower = 0;
        double launcherTrigger = 0;
        boolean intakeOn = false;

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
            launcherFlywheel.setPower(launchTargetTicks);
        }

        if (gamepad1.y) {
            triggerPower = 0.75;
        }

        double ticksPerRevolution = 537.7;

        // Calculate positions
        double augPos3 = ticksPerRevolution / 6; // 60°
        double pos2 = ticksPerRevolution / 3; // 120°
        double augPos1 = ticksPerRevolution / 2; // 180°
        double pos3 = (ticksPerRevolution * 2) / 3; // 240°
        double augPos2 = (ticksPerRevolution * 5) / 6; // 300°
        double pos1 = 0; // 360°

        double target = 0;

        // D-Pad Right
        if (gamepad1.dpad_right) {
            target = gamepad1.left_bumper ? pos3 : augPos3;
        }
        // D-Pad Up
        if (gamepad1.dpad_up) {
            target = gamepad1.left_bumper ? pos2 : augPos2;
        }
        // D-Pad Left
        if (gamepad1.dpad_left) {
            target = gamepad1.left_bumper ? pos1 : augPos1;
        }

        // If any D-pad button pressed, move motor
        if (gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_left) {
         sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         sorter.setTargetPosition((int) target);
         sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        }

        String targetPattern = "GPP";
        String storePattern = "GPP";

        if (gamepad1.dpad_down) {
            if (storePattern == "GPP") {
                if (targetPattern == "GPP") {
                    initiateLuanchSequence(augPos1, augPos2, augPos3);
                } else if (targetPattern == "PGP") {
                    initiateLuanchSequence(augPos2, augPos3, augPos1);
                } else if (targetPattern = "PPG") {
                    initiateLuanchSequence(augPos3, augPos2, augPos1);
                }
            } else if (storePattern == "PGP") {
                if (targetPattern == "GPP") {
                    initiateLuanchSequence(augPos2, augPos1, augPos3);
                } else if (targetPattern == "PGP") {
                    initiateLuanchSequence(augPos1, augPos2, augPos3);
                } else if (targetPattern = "PPG") {
                    initiateLuanchSequence(augPos3, augPos1, augPos2);
                }
            } else if (targetPattern == "PPG") {
                if (targetPattern == "GPP") {
                    initiateLuanchSequence(augPos3, augPos2, augPos1);
                } else if (targetPattern == "PGP") {
                    initiateLuanchSequence(augPos1, augPos3, augPos2);
                } else if (targetPattern = "PPG") {
                    initiateLuanchSequence(augPos1, augPos2, augPos3);
                }
            }
        }

        void initiateLuanchSequence(double targetAugPos1, double targetAugPos2, double targetAugPos3) {
            launcherFlywheel.setPower(1.0);
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sorter.setTargetPosition((int) targetAugPos1);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.3);
            launcherElevator.setPower(1.0);
            sleep(2000);
            launcherElevator.setPower(0);
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sorter.setTargetPosition((int) targetAugPos2);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.3);
            launcherElevator.setPower(1.0);
            sleep(2000);
            launcherElevator.setPower(0);
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sorter.setTargetPosition((int) targetAugPos3);
            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorter.setPower(0.3);
            launcherElevator.setPower(1.0);
            sleep(2000);
            launcherElevator.setPower(0);
            launcherFlywheel.setPower(0);
        }

        telemetry.addData("Sorter Position", sorter.getCurrentPosition());
        telemetry.addData("Sorter Target", sorter.getTargetPosition());
        telemetry.addData("Sorter Busy", sorter.isBusy());

        if (sorter.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !sorter.isBusy()) {
            sorter.setPower(0);
            sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set both launchers to that power
        launcherFlywheel.setPower(triggerPower);
        launcherElevator.setPower(-launcherTrigger);

        telemetry.addData("Trigger", triggerPower);
        telemetry.addData("Launcher1 Power", launcherFlywheel.getPower());
        telemetry.addData("Launcher2 Power", launcherElevator.getPower());
        // (Rest of telemetry and loop logic unchanged)
    }

    @Override
    public void stop() {
        // Stop motors when TeleOp ends
        launcherFlywheel.setPower(0.0);
        launcherElevator.setPower(0.0);
    }
}
