package org.firstinspires.ftc.teamcode.autonomous.search;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

/**
 * Closed-loop vision servo — replaces hard-coded driveLineToX.
 * Drives with mecanum powers computed from blob bearing + range, not trajectories.
 * Intake stays on; exits when close enough for intake or blob lost.
 *
 * This is why prior autos struggled: Actions.runBlocking blocks vision polling.
 * This Action IS the poll loop — it calls setDrivePowers every iteration.
 */
@Config
public class VisionApproachAction implements Action {

    // ---- Tunables (Dashboard) ----
    public static double Kp_bearing = 0.035;   // power per degree (≈0.7 at 20°)
    public static double Kp_range   = 0.055;   // power per inch error
    public static double Kd_bearing = 0.001;
    public static double MAX_FWD = 0.55;
    public static double MAX_STRAFE = 0.45;
    public static double MAX_TURN = 0.40;      // rad/s normalized as strafe? we use turn power
    public static double TARGET_RANGE_IN = 5.0; // intake succeeds at ~4-6"
    public static double BEARING_TOL_DEG = 4.0;
    public static double RANGE_TOL_IN = 2.0;
    public static double LOST_TIMEOUT_S = 0.6; // abort if no blob this long
    public static double SUCCESS_HOLD_S = 0.22; // hold at target to let intake grab
    public static double APPROACH_TIMEOUT_S = 3.2;

    private final MecanumDrive drive;
    private final PollenVision vision;
    private final Runnable intakeOn; // call to keep intake powered
    private final ElapsedTime global = new ElapsedTime();
    private final ElapsedTime lostTimer = new ElapsedTime();
    private final ElapsedTime holdTimer = new ElapsedTime();
    private boolean holding = false;
    private double prevBearing = 0;
    private boolean finished = false;
    private boolean success = false;

    public VisionApproachAction(MecanumDrive drive, PollenVision vision, Runnable intakeOn) {
        this.drive = drive;
        this.vision = vision;
        this.intakeOn = intakeOn;
    }

    public boolean isSuccess() { return success; }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (finished) return false;
        if (global.seconds() == 0) global.reset();
        if (global.seconds() > APPROACH_TIMEOUT_S) { stop(); finished = true; return false; }

        PollenVision.Detection d = vision.poll();
        intakeOn.run();

        if (d == null) {
            if (lostTimer.seconds() > LOST_TIMEOUT_S) {
                packet.put("vision", "lost");
                stop(); finished = true; success = false; return false;
            }
            // Coast while waiting (tiny forward to keep momentum)
            packet.put("vision", "coast");
        } else {
            lostTimer.reset();
            double bearing = d.bearingDeg;
            double range = d.rangeIn;
            double bearingErr = bearing;
            double rangeErr = range - TARGET_RANGE_IN;

            double dBearing = bearing - prevBearing;
            prevBearing = bearing;

            boolean aligned = Math.abs(bearingErr) < BEARING_TOL_DEG;
            boolean ranged  = Math.abs(rangeErr) < RANGE_TOL_IN && range < 12;
            packet.put("bearing", bearingErr);
            packet.put("range", range);
            packet.put("area", d.area);
            packet.put("aligned", aligned);
            packet.put("ranged", ranged);

            if (aligned && ranged) {
                if (!holding) { holding = true; holdTimer.reset(); }
                if (holdTimer.seconds() > SUCCESS_HOLD_S) {
                    stop(); finished = true; success = true; return false;
                }
                // creep forward slowly
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.12, 0), 0));
                return true;
            } else {
                holding = false;
            }

            // PD
            double turn = (Kp_bearing * bearingErr + Kd_bearing * dBearing);
            double fwd  = Kp_range * rangeErr;
            // Convert to mecanum powers: forward + strafe coupling on bearing
            double strafe = Kp_bearing * 0.18 * bearingErr; // tiny strafe to center

            fwd = clamp(fwd, -MAX_FWD, MAX_FWD);
            strafe = clamp(strafe, -MAX_STRAFE, MAX_STRAFE);
            turn = clamp(turn, -MAX_TURN, MAX_TURN);

            // Prioritize turn when large bearing
            if (Math.abs(bearingErr) > 12) fwd *= 0.45;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, -strafe), -turn));
        }

        // keep telemetry overlay
        Canvas c = packet.fieldOverlay();
        c.setStroke("#FFEB3B");
        // draw current pose history done elsewhere

        return true; // keep running
    }

    private void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    @Override
    public void preview(Canvas canvas) {
        canvas.setStroke("#FFEB3B7A");
        canvas.setStrokeWidth(1);
    }
}
