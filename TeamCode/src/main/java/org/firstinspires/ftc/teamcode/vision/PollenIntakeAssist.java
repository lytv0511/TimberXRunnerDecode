package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BasicMecanumDrive;

/**
 * Held-button auto-intake assist for BIOBUZZ pollen / nectar.
 *
 * Uses the webcam detector ({@link PollenVision}) directly — no odometry, no
 * RoadRunner. Bearing closes the loop on turn, range closes the loop on forward.
 * While engaged this class owns the drive motors and the intake group
 * (leftIntake, rightIntake, intakeWheel); the caller must not write those
 * devices at the same time.
 *
 * Wire-up: call {@link #update(boolean)} every loop with the held button state
 * (A button in StudioRunnerTestTeleop). Release stops drive + intake. Timeout
 * forces a stop until the button is released and pressed again.
 */
@Config
public class PollenIntakeAssist {

    // --- Servo loop gains ---
    /** Turn power per degree of bearing error. */
    public static double KP_BEARING = 0.02;
    /** Forward power per inch of range error. */
    public static double KP_RANGE = 0.08;
    public static double MAX_FWD = 0.55;
    public static double MAX_TURN = 0.55;
    /** Below this bearing error the robot may drive forward. */
    public static double ALIGN_TOL_DEG = 8.0;
    /** Stop driving when the game piece is this close to the intake. */
    public static double STOP_RANGE_IN = 6.0;

    /** -1 = any color, otherwise PollenVision.YELLOW / RED / BLUE. */
    public static int TARGET_COLOR = -1;

    /** Keep intaking briefly through brief detection dropouts. */
    public static double TARGET_LOST_MS = 500;
    public static double TIMEOUT_S = 10.0;
    /** Optional slow sweep after a long lost-target period. */
    public static boolean SCAN_WHEN_LOST = false;
    public static double SCAN_TURN_POWER = 0.25;

    // --- Intake powers (match StudioRunnerTestTeleop's RT defaults) ---
    public static double LEFT_INTAKE_POWER = 1.0;
    public static double RIGHT_INTAKE_POWER = -1.0;
    public static double INTAKE_WHEEL_POWER = 1.0;

    private final BasicMecanumDrive drive;
    private final CRServo leftIntake;
    private final CRServo rightIntake;
    private final DcMotor intakeWheel;
    private final PollenVision vision;

    private boolean active = false;
    private long startMs = 0;
    private long lastSeenMs = 0;
    private double lastBearingDeg = 0;
    private String status = "off";

    public PollenIntakeAssist(BasicMecanumDrive drive,
                              CRServo leftIntake, CRServo rightIntake, DcMotor intakeWheel,
                              PollenVision vision) {
        this.drive = drive;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.intakeWheel = intakeWheel;
        this.vision = vision;
    }

    /**
     * Advance the assist. Call once per loop.
     *
     * @param buttonHeld true while the driver holds A
     * @return short status string for telemetry
     */
    public String update(boolean buttonHeld) {
        if (!buttonHeld) {
            if (active) stopAll();
            active = false;
            status = "off";
            return status;
        }

        long now = System.currentTimeMillis();
        if (!active) {
            active = true;
            startMs = now;
            lastSeenMs = now;
        }

        if (now - startMs > TIMEOUT_S * 1000.0) {
            stopAll();
            status = "timeout - release A to reset";
            return status;
        }

        PollenVision.Detection target = vision.poll(TARGET_COLOR);

        if (target != null) {
            lastSeenMs = now;
            lastBearingDeg = target.bearingDeg;

            double turn = clamp(KP_BEARING * target.bearingDeg, -MAX_TURN, MAX_TURN);
            double forward = 0;
            // Rotate to align first; then creep in on range.
            if (Math.abs(target.bearingDeg) <= ALIGN_TOL_DEG) {
                forward = clamp(KP_RANGE * (target.rangeIn - STOP_RANGE_IN), 0, MAX_FWD);
            }

            drive.setDrivePowers(forward, 0, turn);
            setIntake(true);

            if (target.rangeIn <= STOP_RANGE_IN && Math.abs(target.bearingDeg) <= ALIGN_TOL_DEG) {
                status = String.format("at intake - holding (%s)", PollenVision.colorName(target.color));
            } else {
                status = String.format("tracking %s R=%.1f B=%.0f",
                        PollenVision.colorName(target.color), target.rangeIn, target.bearingDeg);
            }
            return status;
        }

        // No target this frame.
        if (now - lastSeenMs <= TARGET_LOST_MS) {
            double turn = clamp(KP_BEARING * lastBearingDeg, -MAX_TURN, MAX_TURN);
            drive.setDrivePowers(0, 0, turn);
            setIntake(true);
            status = "target lost (brief)";
        } else if (SCAN_WHEN_LOST) {
            drive.setDrivePowers(0, 0, SCAN_TURN_POWER);
            setIntake(true);
            status = "scanning";
        } else {
            drive.stop();
            setIntake(false);
            status = "no target";
        }
        return status;
    }

    public boolean isActive() { return active; }
    public String getStatus() { return status; }

    /** Stop drive and intake; safe to call on OpMode shutdown. */
    public void stopAll() {
        drive.stop();
        setIntake(false);
    }

    private void setIntake(boolean on) {
        leftIntake.setPower(on ? LEFT_INTAKE_POWER : 0);
        rightIntake.setPower(on ? RIGHT_INTAKE_POWER : 0);
        intakeWheel.setPower(on ? INTAKE_WHEEL_POWER : 0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
