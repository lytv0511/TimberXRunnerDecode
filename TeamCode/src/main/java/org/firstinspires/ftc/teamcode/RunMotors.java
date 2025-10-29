package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RunMotors", group="TeleOp")
public class RunMotors extends OpMode {

    private DcMotor launcherRight;
    private DcMotor launcherLeft;

    @Override
    public void init() {
        // Map motors from the configuration
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");

        // Adjust direction if needed
        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Right trigger on gamepad1 goes from 0.0 to 1.0
        double triggerPower = gamepad1.right_trigger;

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
        launcherLeft.setPower(triggerPower);

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
