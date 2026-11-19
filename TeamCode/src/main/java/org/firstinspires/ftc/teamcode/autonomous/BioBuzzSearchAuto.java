package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Search-style autonomous on the Basic Mecanum config.
 *
 * The original version used PollenVision + SearchPlanner, but this config has
 * no camera, so it runs a timed sweep-and-intake pattern instead. "Collected"
 * is a timed estimate — there is no sensor on this robot to confirm a ball.
 * The vision/search classes are still in the tree for a camera-equipped config.
 */
@Config
@Autonomous(name = "BioBuzzSearchAuto", group = "Autonomous")
public class BioBuzzSearchAuto extends BioBuzzAutoBase {

    public static boolean ALLIANCE_IS_BLUE = false;
    public static int TARGET_POLLEN = 5;
    public static double TIMEOUT_SEC = 22.0; // leaves time to shoot/stop inside 30s
    public static double BACKOFF_S = 0.35;
    public static double HOP_S = 0.45;
    public static double SWEEP_S = 0.35;

    @Override
    protected double getAllianceSign() {
        return ALLIANCE_IS_BLUE ? BioBuzzConstants.BLUE_SIGN : BioBuzzConstants.RED_SIGN;
    }

    @Override
    protected void runAuto() {
        backOffWall(BACKOFF_S);

        ElapsedTime timer = new ElapsedTime();
        int collected = 0;
        boolean sweepRight = true;

        while (opModeIsActive() && timer.seconds() < TIMEOUT_SEC && collected < TARGET_POLLEN) {
            // Short intake hop, then sweep the other direction to cover the area.
            pollen.nudgeIntake(HOP_S);
            turnBy(SWEEP_S, BioBuzzConstants.DRIVE_TURN_POWER * (sweepRight ? 1.0 : -1.0));
            sweepRight = !sweepRight;

            collected++;
            ballCount = collected;
            telemetry.addData("Collected (timed estimate)", collected);
            telemetry.update();
        }

        intakeOff();
        telemetry.addData("Acquired", collected);
        telemetry.update();

        waitSeconds(0.5);
        shoot(Math.min(collected, BioBuzzConstants.AUTO_MAX_POLLEN));
    }
}
