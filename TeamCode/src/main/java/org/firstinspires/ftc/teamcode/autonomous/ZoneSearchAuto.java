package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

/**
 * Zone-based autonomous for BIOBUZZ.
 * Drives to configurable zones, scans with camera, approaches pollen, activates intake, eats ball.
 *
 * How it works:
 *   1. Backoff 3.5" from wall (only hard-coded move)
 *   2. Drive to zone A (Dashboard: ZONE_A_X/Y)
 *   3. Scan with camera (heading sweep + PollenVision)
 *   4. If pollen found -> approach with intake ON -> eat
 *   5. Move to zone B, zone C, repeat
 *   6. Wrap around to zone A after last zone
 *   7. Stop on timeout or target count
 *
 * Calibration prerequisite:
 *   Run VisionCalibration TeleOp first. Set PollenVision.COLOR_MODE, FOCAL_PX, etc.
 *   until detection works at known distances. Then deploy this auto.
 *
 * Dashboard tunables:
 *   ALLIANCE_IS_BLUE  — mirror zone Y for Blue alliance
 *   ZONE_A/B/C_X/Y   — zone coordinates (inches, start-relative)
 *   ZONE_COUNT        — how many zones to visit (1-3)
 *   TARGET_POLLEN     — how many balls to collect
 *   TIMEOUT_SEC       — autonomous timeout (leave 4s buffer for 30s auto)
 *   SAFE_BACKOFF_IN   — clear wall before search
 */
@Config
@Autonomous(name = "ZoneSearchAuto", group = "Autonomous")
public class ZoneSearchAuto extends BioBuzzAutoBase {

    public static boolean ALLIANCE_IS_BLUE = false;

    // Zone A (first zone to visit)
    public static double ZONE_A_X = 24;
    public static double ZONE_A_Y = 48;
    public static double ZONE_A_HDG = 90;  // degrees

    // Zone B (second zone)
    public static double ZONE_B_X = 48;
    public static double ZONE_B_Y = 48;
    public static double ZONE_B_HDG = 45;

    // Zone C (third zone)
    public static double ZONE_C_X = 48;
    public static double ZONE_C_Y = -48;
    public static double ZONE_C_HDG = -45;

    public static int ZONE_COUNT = 2;        // 1, 2, or 3
    public static int TARGET_POLLEN = 3;
    public static double TIMEOUT_SEC = 26.0;
    public static double SAFE_BACKOFF_IN = 3.5;

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
        // 1) Backoff from wall
        if (SAFE_BACKOFF_IN > 0.5) {
            Pose2d cur = drive.localizer.getPose();
            Actions.runBlocking(drive.actionBuilder(cur)
                    .lineToX(cur.position.x - SAFE_BACKOFF_IN).build());
        }

        // 2) Build zone list from Dashboard fields
        List<Pose2d> zones = new ArrayList<>();
        double sign = getAllianceSign();
        zones.add(new Pose2d(ZONE_A_X, ZONE_A_Y * sign, Math.toRadians(ZONE_A_HDG * sign)));
        if (ZONE_COUNT >= 2)
            zones.add(new Pose2d(ZONE_B_X, ZONE_B_Y * sign, Math.toRadians(ZONE_B_HDG * sign)));
        if (ZONE_COUNT >= 3)
            zones.add(new Pose2d(ZONE_C_X, ZONE_C_Y * sign, Math.toRadians(ZONE_C_HDG * sign)));

        telemetry.addData("Zones", zones.size());
        telemetry.addData("Target", TARGET_POLLEN);
        telemetry.update();

        // 3) Run zone search — drive, scan, approach, intake, repeat
        int got = runZoneSearch(zones, TARGET_POLLEN, TIMEOUT_SEC);

        // 4) Done — stop
        telemetry.addData("Acquired", got);
        telemetry.addData("Final", drive.localizer.getPose());
        telemetry.update();

        waitSeconds(0.5);
        intakeOff();
    }
}
