package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoBase;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzConstants;

/**
 * Red autonomous on the Basic Mecanum config.
 * Open-loop and timed: shoot preload, collect a pile, return, shoot again.
 * No launcher/sorter/elevator/camera — only the 8 devices in the new config.
 */
@Config
@Autonomous(name = "StudioTestAutoRed", group = "Autonomous")
public class StudioTestAutoRed extends BioBuzzAutoBase {

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
        return BioBuzzConstants.RED_SIGN;
    }

    @Override
    protected void runAuto() {
        // 1) Clear the wall and shoot the preload.
        backOffWall(BACKOFF_S);
        shoot(PRELOAD_BALLS);

        // 2) Turn/strafe toward the pollen, approach while intaking a pile.
        turnBy(TURN_TO_INTAKE_S, BioBuzzConstants.DRIVE_TURN_POWER);
        strafeBy(STRAFE_TO_INTAKE_S, BioBuzzConstants.DRIVE_STRAFE_POWER);
        driveForward(APPROACH_S);
        pollen.intakePile();

        // 3) Return to the shooting area and fire what we collected.
        strafeBy(RETURN_STRAFE_S, -BioBuzzConstants.DRIVE_STRAFE_POWER);
        driveForward(RETURN_S);
        turnBy(TURN_TO_SHOOT_S, BioBuzzConstants.DRIVE_TURN_POWER);
        shoot(ballCount);

        intakeOff();
    }
}
