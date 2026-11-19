package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoBase;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzConstants;

/**
 * Blue autonomous on the Basic Mecanum config.
 * Mirror of Red: getAllianceSign() flips strafe and turn commands.
 */
@Config
@Autonomous(name = "StudioTestAutoBlue", group = "Autonomous")
public class StudioTestAutoBlue extends BioBuzzAutoBase {

    public static int PRELOAD_BALLS = 3;
    public static double BACKOFF_S = 0.35;
    public static double TURN_TO_INTAKE_S = 0.80;
    public static double STRAFE_TO_INTAKE_S = 0.50;
    public static double APPROACH_S = 0.60;
    public static double RETURN_STRAFE_S = 0.50;
    public static double RETURN_S = 0.60;
    public static double TURN_TO_SHOOT_S = 0.80;

    @Override
    protected double getAllianceSign() {
        return BioBuzzConstants.BLUE_SIGN;
    }

    @Override
    protected void runAuto() {
        backOffWall(BACKOFF_S);
        shoot(PRELOAD_BALLS);

        turnBy(TURN_TO_INTAKE_S, BioBuzzConstants.DRIVE_TURN_POWER);
        strafeBy(STRAFE_TO_INTAKE_S, BioBuzzConstants.DRIVE_STRAFE_POWER);
        driveForward(APPROACH_S);
        pollen.intakePile();

        strafeBy(RETURN_STRAFE_S, -BioBuzzConstants.DRIVE_STRAFE_POWER);
        driveForward(RETURN_S);
        turnBy(TURN_TO_SHOOT_S, BioBuzzConstants.DRIVE_TURN_POWER);
        shoot(ballCount);

        intakeOff();
    }
}
