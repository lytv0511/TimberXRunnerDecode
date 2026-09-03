package org.firstinspires.ftc.teamcode.autonomous.search;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

import java.util.List;

/**
 * Zone-based autonomous: drive to each zone, scan, approach with intake, eat ball.
 * Simpler than SearchAndIntakeAction — no probability decay, just ordered zones.
 *
 * Flow per zone:
 *   DRIVING → SCAN → APPROACH → INTAKING → next zone
 *
 * Opportunistic: while driving, if pollen visible < 30" and |bearing| < 22°,
 * abort drive and chase immediately.
 */
@Config
public class ZoneSearchAction implements Action {

    // ---- Tunables ----
    public static double OPPORTUNISTIC_RANGE_IN = 30;
    public static double OPPORTUNISTIC_BEARING_DEG = 22;
    public static double INTAKE_HOLD_S = 0.5;

    private final MecanumDrive drive;
    private final PollenVision vision;
    private final List<Pose2d> zones;
    private final Runnable intakeOn;
    private final Runnable intakeOff;
    private final int targetCount;
    private final double timeoutS;

    private final ElapsedTime global = new ElapsedTime();
    private int acquired = 0;
    private int zoneIdx = 0;
    private State state = State.DRIVING;
    private Action curAction = null;
    private boolean driveActionBuilt = false;

    private enum State { DRIVING, SCAN, APPROACH, INTAKING }

    public ZoneSearchAction(MecanumDrive drive, PollenVision vision, List<Pose2d> zones,
                            Runnable intakeOn, Runnable intakeOff,
                            int targetCount, double timeoutS) {
        this.drive = drive;
        this.vision = vision;
        this.zones = zones;
        this.intakeOn = intakeOn;
        this.intakeOff = intakeOff;
        this.targetCount = targetCount;
        this.timeoutS = timeoutS;
    }

    public int getAcquired() { return acquired; }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (global.seconds() == 0) global.reset();

        if (global.seconds() > timeoutS || acquired >= targetCount) {
            finish(packet, "done");
            return false;
        }
        if (zones.isEmpty()) { finish(packet, "no zones"); return false; }

        packet.put("state", state.name());
        packet.put("acquired", acquired);
        packet.put("zone", zoneIdx + "/" + zones.size());
        Pose2d curPose = drive.localizer.getPose();

        switch (state) {
            case DRIVING: {
                // Build trajectory on first entry to DRIVING for this zone
                if (!driveActionBuilt) {
                    // Opportunistic check before building trajectory
                    PollenVision.Detection d = vision.poll();
                    if (d != null && d.rangeIn < OPPORTUNISTIC_RANGE_IN) {
                        curAction = new VisionApproachAction(drive, vision, intakeOn);
                        state = State.APPROACH;
                        break;
                    }
                    Pose2d target = zones.get(zoneIdx);
                    curAction = drive.actionBuilder(curPose)
                            .splineTo(target.position, target.heading.toDouble())
                            .build();
                    driveActionBuilt = true;
                }

                // Poll vision while driving
                PollenVision.Detection d = vision.poll();
                if (d != null && d.rangeIn < OPPORTUNISTIC_RANGE_IN
                        && Math.abs(d.bearingDeg) < OPPORTUNISTIC_BEARING_DEG) {
                    curAction = new VisionApproachAction(drive, vision, intakeOn);
                    state = State.APPROACH;
                    break;
                }

                // Step trajectory
                if (curAction == null) {
                    // No trajectory — go straight to scan
                    curAction = new ScanAction(drive, vision);
                    state = State.SCAN;
                    break;
                }
                boolean still = curAction.run(packet);
                if (!still) {
                    curAction = new ScanAction(drive, vision);
                    state = State.SCAN;
                }
                return true;
            }
            case SCAN: {
                if (curAction == null) curAction = new ScanAction(drive, vision);
                boolean still = curAction.run(packet);
                if (!still) {
                    boolean found = false;
                    if (curAction instanceof ScanAction) found = ((ScanAction) curAction).foundTarget();
                    if (found) {
                        curAction = new VisionApproachAction(drive, vision, intakeOn);
                        state = State.APPROACH;
                    } else {
                        advanceZone();
                    }
                }
                return true;
            }
            case APPROACH: {
                if (curAction == null) curAction = new VisionApproachAction(drive, vision, intakeOn);
                boolean still = curAction.run(packet);
                if (!still) {
                    boolean success = false;
                    if (curAction instanceof VisionApproachAction)
                        success = ((VisionApproachAction) curAction).isSuccess();
                    if (success) {
                        state = State.INTAKING;
                        curAction = null;
                    } else {
                        advanceZone();
                    }
                }
                return true;
            }
            case INTAKING: {
                if (curAction == null) {
                    ElapsedTime hold = new ElapsedTime();
                    hold.reset();
                    curAction = new Action() {
                        @Override public boolean run(@NonNull TelemetryPacket p) {
                            intakeOn.run();
                            return hold.seconds() < INTAKE_HOLD_S;
                        }
                        @Override public void preview(Canvas c) {}
                    };
                }
                boolean still = curAction.run(packet);
                if (!still) {
                    acquired++;
                    packet.put("ate", acquired);
                    advanceZone();
                }
                return true;
            }
        }
        return true;
    }

    private void advanceZone() {
        zoneIdx++;
        if (zoneIdx >= zones.size()) zoneIdx = 0;
        curAction = null;
        driveActionBuilt = false;
        state = State.DRIVING;
    }

    private void finish(TelemetryPacket packet, String reason) {
        intakeOff.run();
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        packet.put("search", reason + " " + acquired + "/" + targetCount);
    }

    @Override
    public void preview(Canvas canvas) {}
}
