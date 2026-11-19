package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Bench for BIOBUZZ drills on the Basic Mecanum config.
 * All movements are timed — no field coordinates or camera.
 */
@Config
@Autonomous(name = "PreseasonBioBuzzBench", group = "Preseason")
public class PreseasonBioBuzzBench extends BioBuzzAutoBase {

    public static int MODE = 0; // 0=floor, 1=line, 2=pile, 3=wall, 4=corner, 5=timed nav, 6=timed search
    public static double WAYPOINT_1_S = 0.60;
    public static double WAYPOINT_2_S = 0.80;
    public static double WAYPOINT_3_S = 0.60;
    public static int SEARCH_TARGET = 3;
    public static double SEARCH_TIMEOUT_S = 12.0;

    @Override
    protected double getAllianceSign() {
        return BioBuzzConstants.RED_SIGN;
    }

    @Override
    protected void runAuto() {
        telemetry.addData("Bench MODE", MODE);
        telemetry.update();

        switch (MODE) {
            case 0:
                pollen.intakeSingle();
                break;
            case 1:
                pollen.intakeLine(3);
                break;
            case 2:
                pollen.intakePile();
                break;
            case 3:
                pollen.intakeOffWall();
                break;
            case 4:
                pollen.intakeFromCorner();
                break;
            case 5:
                pollen.navigateAndIntake(WAYPOINT_1_S, WAYPOINT_2_S, WAYPOINT_3_S);
                break;
            case 6:
                // Timed search pattern (no camera on this config).
                ElapsedTime timer = new ElapsedTime();
                int collected = 0;
                boolean right = true;
                while (opModeIsActive() && timer.seconds() < SEARCH_TIMEOUT_S
                        && collected < SEARCH_TARGET) {
                    pollen.nudgeIntake(BioBuzzConstants.SEARCH_HOP_S);
                    turnBy(BioBuzzConstants.SEARCH_SWEEP_S,
                            BioBuzzConstants.DRIVE_TURN_POWER * (right ? 1.0 : -1.0));
                    right = !right;
                    collected++;
                }
                telemetry.addData("Timed search got", collected);
                telemetry.update();
                break;
            default:
                pollen.intakeLine(2);
                driveForward(0.60);
                pollen.intakeOffWall();
                strafeBy(0.50, BioBuzzConstants.DRIVE_STRAFE_POWER);
                pollen.intakeFromCorner();
                break;
        }

        intakeOff();

        telemetry.addData("Balls handled", ballCount);
        telemetry.update();
        waitSeconds(1.0);
    }
}
