package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Open-loop mecanum drive for the Basic Mecanum hardware config:
 *   leftFront, rightFront, leftBack, rightBack (DcMotor).
 *
 * Standard mecanum power mixing; rightFront and rightBack are reversed.
 * No IMU, encoders, odometry, or RoadRunner trajectories — all movement is
 * expressed as timed power commands by the calling OpMode.
 */
public class BasicMecanumDrive {

    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;

    public BasicMecanumDrive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * @param forward +1 drives forward, -1 backward
     * @param strafe  +1 strafes right, -1 left
     * @param turn    +1 rotates clockwise, -1 counter-clockwise (gamepad right-stick sign)
     */
    public void setDrivePowers(double forward, double strafe, double turn) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0
        );

        leftFront.setPower((forward + strafe + turn) / denominator);
        leftBack.setPower((forward - strafe + turn) / denominator);
        rightFront.setPower((forward - strafe - turn) / denominator);
        rightBack.setPower((forward + strafe - turn) / denominator);
    }

    public void stop() {
        setDrivePowers(0, 0, 0);
    }
}
