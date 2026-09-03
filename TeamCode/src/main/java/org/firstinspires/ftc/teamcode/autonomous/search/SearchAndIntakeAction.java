package org.firstinspires.ftc.teamcode.autonomous.search;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

/**
 * High-level autonomous: probability → drive → scan → vision-servo → intake → loop.
 * No hard-coded paths beyond SearchPlanner cells. This replaces the old -51/turn sequences.
 */
public class SearchAndIntakeAction implements Action {

    private final MecanumDrive drive;
    private final PollenVision vision;
    private final SearchPlanner planner;
    private final Runnable intakeOn;
    private final Runnable intakeOff;
    private final int targetCount;
    private final double timeoutS;
    private final ElapsedTime global = new ElapsedTime();

    private int acquired = 0;
    private State state = State.PICK_NEXT;
    private SearchCell curCell = null;
    private Action curAction = null;

    private enum State { PICK_NEXT, DRIVING, SCANNING, APPROACHING, INTAKING }

    public SearchAndIntakeAction(MecanumDrive drive, PollenVision vision, SearchPlanner planner,
                                 Runnable intakeOn, Runnable intakeOff,
                                 int targetCount, double timeoutS) {
        this.drive = drive; this.vision = vision; this.planner = planner;
        this.intakeOn = intakeOn; this.intakeOff = intakeOff;
        this.targetCount = targetCount; this.timeoutS = timeoutS;
    }

    public int getAcquired() { return acquired; }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (global.seconds() == 0) global.reset();
        if (global.seconds() > timeoutS || acquired >= targetCount) {
            intakeOff.run();
            drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(new com.acmerobotics.roadrunner.Vector2d(0,0),0));
            packet.put("search", "done " + acquired + "/" + targetCount);
            return false;
        }

        packet.put("state", state.name());
        packet.put("acquired", acquired);
        Pose2d curPose = drive.localizer.getPose();

        switch (state) {
            case PICK_NEXT: {
                // First, opportunistic check: is pollen already in view? Skip driving.
                PollenVision.Detection d = vision.poll();
                if (d != null && d.rangeIn < 24) {
                    // Go straight to approach — save time
                    VisionApproachAction approach = new VisionApproachAction(drive, vision, intakeOn);
                    curAction = approach;
                    state = State.APPROACHING;
                    break;
                }
                curCell = planner.next(curPose);
                if (curCell == null) return false;
                packet.put("nextCell", curCell.name);
                // Non-blocking trajectory: wrap as Action and step it
                // Use simple spline/line — distance short, so line is fine for search hops
                Pose2d target = curCell.pose;
                // Build a short trajectory to target position (keep heading of cell)
                // We run it via Actions.runBlocking off-thread would block vision → instead step via Action
                // Here we create an Action and delegate
                curAction = drive.actionBuilder(curPose).splineTo(target.position, target.heading.toDouble()).build();
                state = State.DRIVING;
                break;
            }
            case DRIVING: {
                // Keep polling vision while driving — opportunistic early exit
                PollenVision.Detection d = vision.poll();
                if (d != null && d.rangeIn < 30 && Math.abs(d.bearingDeg) < 22) {
                    // Abort drive, chase visible pollen
                    // Stopping drive: curAction will be abandoned; next iteration will servo
                    // We need to stop motors (FollowTrajectoryAction stops on interruption via returning false)
                    // Cheapest: just switch state; next run will not resume curAction
                    curAction = new VisionApproachAction(drive, vision, intakeOn);
                    state = State.APPROACHING;
                    break;
                }
                if (curAction == null) { state = State.SCANNING; break; }
                boolean still = curAction.run(packet);
                if (!still) {
                    // Arrived — scan
                    ScanAction scan = new ScanAction(drive, vision);
                    curAction = scan;
                    state = State.SCANNING;
                }
                return true; // keep driving
            }
            case SCANNING: {
                if (curAction == null) { curAction = new ScanAction(drive, vision); }
                boolean still = curAction.run(packet);
                if (!still) {
                    boolean found = false;
                    if (curAction instanceof ScanAction) found = ((ScanAction) curAction).foundTarget();
                    planner.markResult(curCell, found);
                    if (found) {
                        curAction = new VisionApproachAction(drive, vision, intakeOn);
                        state = State.APPROACHING;
                    } else {
                        // No pollen here — try next cell, decay boosts
                        planner.decayBoosts();
                        state = State.PICK_NEXT;
                    }
                }
                return true;
            }
            case APPROACHING: {
                if (curAction == null) curAction = new VisionApproachAction(drive, vision, intakeOn);
                boolean still = curAction.run(packet);
                if (!still) {
                    boolean success = false;
                    if (curAction instanceof VisionApproachAction) success = ((VisionApproachAction) curAction).isSuccess();
                    if (success) {
                        state = State.INTAKING;
                        // brief intake hold
                        intakeOn.run();
                        // reuse curAction as timer
                        curAction = null;
                    } else {
                        // Failed approach — mark cell not fruitful from this angle
                        planner.markResult(curCell, false);
                        state = State.PICK_NEXT;
                    }
                }
                return true;
            }
            case INTAKING: {
                // Hold intake 0.5s to ensure roller grabs Pollen (3" ball takes ~0.3s)
                // Use a simple timed hold
                if (curAction == null) {
                    ElapsedTime hold = new ElapsedTime(); hold.reset();
                    curAction = new Action() {
                        @Override public boolean run(@NonNull TelemetryPacket p) {
                            intakeOn.run();
                            return hold.seconds() < 0.45;
                        }
                        @Override public void preview(Canvas c) {}
                    };
                }
                boolean still = curAction.run(packet);
                if (!still) {
                    acquired++;
                    planner.markResult(curCell, true);
                    // Optional: small back-up to clear pile
                    // keep intake on for pile follow-through
                    state = State.PICK_NEXT;
                    curAction = null;
                }
                return true;
            }
        }
        return true;
    }

    @Override public void preview(Canvas canvas) {}
}
