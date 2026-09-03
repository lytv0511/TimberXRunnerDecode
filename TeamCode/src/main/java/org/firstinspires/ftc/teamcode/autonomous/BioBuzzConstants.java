package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

/**
 * Preseason constants for BIOBUZZ (2026-27 CANOPY).
 * Field/scoring not yet revealed — values are tunable placeholders for planning-stage workflows.
 * Update after 2026-09-12 kickoff and V1 manual.
 */
@Config
public final class BioBuzzConstants {
    private BioBuzzConstants() {}

    // --- Pollen (scoring element) ---
    /** Diameter ~3in, same class as DECODE Artifacts. Keep intake tuned to 3in balls. */
    public static double POLLEN_DIAMETER_IN = 3.0;
    /** Approx weight not yet published — tune after AndyMark delivery. */
    public static double POLLEN_DIAMETER_TOL_IN = 0.15;

    // --- Intake (hardware unchanged — leave mapping alone) ---
    public static double INTAKE_POWER = -1.0;
    public static double INTAKE_IDLE_POWER = 0.0;
    public static double ELEVATOR_FEED_POWER = -1.0;
    public static double ELEVATOR_HOLD_POWER = 0.2;
    // Sorter 3-slot positions (ticks) — DECODE heritage, keep for now until BIOBUZZ sort/no-sort decided
    public static double TICKS_PER_REV = 537.7;
    public static double SORTER_POS_0 = 0;
    public static double SORTER_POS_1 = 537.7 / 3.0;          // 120°
    public static double SORTER_POS_2 = 537.7 * 2.0 / 3.0;     // 240°
    public static double SORTER_POWER = 0.30;

    // --- Launcher (DECODE — likely unused in BIOBUZZ bulk-transfer, keep for reference) ---
    public static double LAUNCHER_TARGET_VEL = 1680; // ticks/s
    public static double LAUNCHER_VEL_TOL = 50;
    public static double LAUNCHER_SPINUP_MS = 500;

    // --- Drive planning (tune after field CAD) ---
    /** Preseason test distances — replace with field coordinates post-kickoff. */
    public static double NUDGE_SHORT_IN = 1.25;
    public static double NUDGE_LONG_IN = 1.50;
    public static double WALL_APPROACH_IN = 7.0;
    public static double CORNER_STRAFE_IN = 10.0;
    public static double PILE_SPACING_IN = 6.0;

    // --- Autonomous timing ---
    public static double AUTO_INTAKE_TIMEOUT_S = 8.0;
    public static double AUTO_FEED_S = 0.25;
    public static int AUTO_MAX_POLLEN = 3; // BIOBUZZ preview: intake multiple

    // --- Alliance mirroring ---
    public static double BLUE_SIGN = -1.0; // Blue mirrors Red heading
    public static double RED_SIGN = 1.0;
}
