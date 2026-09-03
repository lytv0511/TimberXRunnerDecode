package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.search.SearchAndIntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.search.SearchPlanner;

/**
 * Real autonomous — NO hard-coded field paths.
 * Robot moves to high-probability pollen cells and vision-servos to intake.
 *
 * How it solves the historic struggle:
 *  - Old autos blocked vision inside Actions.runBlocking trajectories → missed pollen.
 *  - This auto interleaves: SearchPlanner picks next cell → short spline → ScanAction sweeps
 *    while polling PollenVision every 15ms → VisionApproachAction servos with mecanum powers.
 *  - Probability field decays visited cells, boosts neighbors (piles), re-ranks by distance.
 *
 * Tune in Dashboard: BioBuzzConstants, PollenVision, VisionApproachAction, SearchPlanner.
 * Calibrate PollenVision.FOCAL_PX once (12" test). Then run PreseasonBioBuzzBench MODE 3/4 for walls/corners.
 */
@Config
@Autonomous(name = "BioBuzzSearchAuto", group = "Autonomous")
public class BioBuzzSearchAuto extends BioBuzzAutoBase {

    // ---- Dashboard tunables ----
    public static boolean ALLIANCE_IS_BLUE = false;
    public static int TARGET_POLLEN = 5;
    public static double TIMEOUT_SEC = 26.0; // leave 4s buffer for 30s auto
    public static double SAFE_BACKOFF_IN = 3.5; // clear wall before search

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(0, 0, 0);
    }

    @Override
    protected double getAllianceSign() {
        return ALLIANCE_IS_BLUE ? BioBuzzConstants.BLUE_SIGN : BioBuzzConstants.RED_SIGN;
    }

    @Override
    protected void runAuto() {
        // 1) Safe nudge off wall — only hard-coded move (field-agnostic, <6in)
        if (SAFE_BACKOFF_IN > 0.5) {
            Pose2d cur = drive.localizer.getPose();
            Actions.runBlocking(drive.actionBuilder(cur).lineToX(cur.position.x - SAFE_BACKOFF_IN).build());
            telemetry.addData("Backoff", SAFE_BACKOFF_IN);
            telemetry.update();
        }

        // 2) Optional: if you know your alliance station is at known pose, seed planner center nearer to you.
        //    Otherwise default planner (walls/corners/lines) is fine for preseason.

        // 3) Search — this is the real autonomous, not a trajectory list
        telemetry.addLine("Starting vision search...");
        telemetry.addData("Target", TARGET_POLLEN);
        telemetry.update();

        int got = runSearchAndIntake(TARGET_POLLEN, TIMEOUT_SEC);

        // 4) Score — BIOBUZZ base-station deposit not yet revealed.
        //    For preseason, just stop with pollen held. Post-kickoff: replace with:
        //    drive to baseStation pose (from planner) → outtake (elevator/reverse intake)
        telemetry.addData("Acquired", got);
        telemetry.addData("Final", drive.localizer.getPose());
        telemetry.update();

        // 5) Hold for telemetry before stop
        waitSeconds(0.8);
        intakeOff();
    }
}
