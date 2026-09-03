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
 * In-place scan at a SearchCell — sweeps heading ±SWEEP_DEG while polling vision.
 * Exits early when blob found. Used between SearchPlanner hops.
 */
@Config
public class ScanAction implements Action {
    public static double SWEEP_DEG = 35;
    public static double SWEEP_POWER = 0.22; // turn power
    public static double DWELL_S = 2.0; // max scan time if no target

    private final MecanumDrive drive;
    private final PollenVision vision;
    private final ElapsedTime t = new ElapsedTime();
    private boolean started = false;
    private boolean found = false;

    public ScanAction(MecanumDrive drive, PollenVision vision) {
        this.drive = drive; this.vision = vision;
    }
    public boolean foundTarget() { return found; }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!started) { started = true; t.reset(); }
        // Poll each iteration
        PollenVision.Detection d = vision.poll();
        if (d != null && d.area > PollenVision.MIN_AREA_PX) {
            found = true;
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
            packet.put("scan", "found bearing " + String.format("%.1f", d.bearingDeg));
            return false; // done — let VisionApproach take over
        }
        if (t.seconds() > DWELL_S) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
            packet.put("scan", "timeout");
            return false;
        }
        // Oscillate: turn right then left
        double phase = (t.seconds() / DWELL_S) * 2 * Math.PI;
        // simple sweep: sin wave
        double turn = Math.sin(phase) * SWEEP_POWER;
        // add slow forward bias near walls to peek around border
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.05, 0), turn));
        packet.put("scan t", t.seconds());
        return true;
    }

    @Override public void preview(Canvas canvas) {}
}
