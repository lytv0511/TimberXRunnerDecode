package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Zone-based autonomous on the Basic Mecanum config.
 *
 * The old version drove to inch coordinates using RoadRunner and scanned with
 * a camera. With no localizer or camera on this config, zones are expressed as
 * timed drive/turn segments and "acquired" is a timed estimate.
 */
@Config
@Autonomous(name = "ZoneSearchAuto", group = "Autonomous")
public class ZoneSearchAuto extends BioBuzzAutoBase {

    public static boolean ALLIANCE_IS_BLUE = false;

    // Timed zone segments (seconds) — replace with measured values on the tiles.
    public static double ZONE_A_DRIVE_S = 0.80;
    public static double ZONE_A_TURN_S = 0.40;
    public static double ZONE_B_DRIVE_S = 0.70;
    public static double ZONE_B_TURN_S = 0.50;
    public static double ZONE_C_DRIVE_S = 0.90;
    public static double ZONE_C_TURN_S = 0.40;

    public static int ZONE_COUNT = 2;        // 1, 2, or 3
    public static int TARGET_POLLEN = 3;
    public static double TIMEOUT_SEC = 22.0;
    public static double BACKOFF_S = 0.35;

    @Override
    protected double getAllianceSign() {
        return ALLIANCE_IS_BLUE ? BioBuzzConstants.BLUE_SIGN : BioBuzzConstants.RED_SIGN;
    }

    @Override
    protected void runAuto() {
        backOffWall(BACKOFF_S);

        double[] driveSec = {ZONE_A_DRIVE_S, ZONE_B_DRIVE_S, ZONE_C_DRIVE_S};
        double[] turnSec = {ZONE_A_TURN_S, ZONE_B_TURN_S, ZONE_C_TURN_S};
        int zones = Math.max(1, Math.min(3, ZONE_COUNT));

        telemetry.addData("Zones", zones);
        telemetry.addData("Target", TARGET_POLLEN);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        int collected = 0;
        while (opModeIsActive() && timer.seconds() < TIMEOUT_SEC && collected < TARGET_POLLEN) {
            for (int i = 0; i < zones && opModeIsActive() && collected < TARGET_POLLEN; i++) {
                pollen.intakeOn();
                driveForward(driveSec[i]);
                turnBy(turnSec[i], BioBuzzConstants.DRIVE_TURN_POWER);
                pollen.nudgeIntake(BioBuzzConstants.NUDGE_SHORT_S);

                collected++;
                ballCount = collected;
                telemetry.addData("Zone", i + 1);
                telemetry.addData("Collected (timed estimate)", collected);
                telemetry.update();
            }
        }

        intakeOff();
        telemetry.addData("Acquired", collected);
        telemetry.update();

        waitSeconds(0.5);
        shoot(Math.min(collected, BioBuzzConstants.AUTO_MAX_POLLEN));
    }
}
