package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

/**
 * Tunables for BIOBUZZ autos running on the Basic Mecanum configuration:
 *   leftFront/rightFront/leftBack/rightBack (DcMotor),
 *   leftIntake/rightIntake (CRServo), intakeWheel, rubber.
 *
 * This config has no IMU, encoders, camera, color sensor, sorter or elevator,
 * so every movement is open-loop and timed. Tune the seconds values on the tiles.
 */
@Config
public final class BioBuzzConstants {
    private BioBuzzConstants() {}

    // --- Scoring element ---
    public static double POLLEN_DIAMETER_IN = 3.0;

    // --- Mechanism powers (same signs as StudioTestTeleop's right-trigger action) ---
    public static double LEFT_INTAKE_POWER = 1.0;
    public static double RIGHT_INTAKE_POWER = -1.0;
    public static double INTAKE_WHEEL_POWER = 1.0;
    public static double RUBBER_POWER = -1.0;

    // --- Open-loop drive powers ---
    public static double DRIVE_FORWARD_POWER = 0.55;
    public static double DRIVE_STRAFE_POWER = 0.45;
    public static double DRIVE_TURN_POWER = 0.40;

    // --- Timed moves (seconds) ---
    public static double BACKOFF_S = 0.35;
    public static double NUDGE_SHORT_S = 0.30;
    public static double NUDGE_LONG_S = 0.50;
    public static double WALL_APPROACH_S = 0.80;
    public static double CORNER_STRAFE_S = 0.70;
    public static double PILE_SPACING_S = 0.45;
    public static double INTAKE_SETTLE_S = 0.20;
    public static double INTAKE_HOLD_S = 0.45;

    // --- Timed search pattern ---
    public static double SEARCH_HOP_S = 0.45;
    public static double SEARCH_SWEEP_S = 0.35;

    // --- Shooting ---
    public static double SHOOT_SPINUP_S = 0.60;
    public static double SHOOT_FEED_S = 0.40;
    public static double SHOOT_GAP_S = 0.15;

    // --- Auto timing ---
    public static double AUTO_TIMEOUT_S = 22.0; // leaves time to shoot/stop inside 30s
    public static int AUTO_MAX_POLLEN = 3;

    // --- Alliance mirroring ---
    public static double BLUE_SIGN = -1.0;
    public static double RED_SIGN = 1.0;
}
