package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Open-loop, timed versions of the preseason BIOBUZZ intake drills.
 * Uses only the Basic Mecanum config devices exposed by BioBuzzAutoBase.
 */
public class PollenWorkflow {

    private final BioBuzzAutoBase op;

    public PollenWorkflow(BioBuzzAutoBase op) {
        this.op = op;
    }

    public void intakeOn() {
        op.intakeOn();
    }

    public void intakeOff() {
        op.intakeOff();
    }

    /** Drive forward briefly with the intake running. */
    public void nudgeIntake(double seconds) {
        op.intakeOn();
        op.driveForward(seconds);
    }

    /** Task 1: single pollen off the floor. */
    public void intakeSingle() {
        nudgeIntake(BioBuzzConstants.NUDGE_SHORT_S);
        op.waitSeconds(BioBuzzConstants.INTAKE_SETTLE_S);
        op.ballCount++;
    }

    /** Task 2: several pollen in a line — timed hops between them. */
    public void intakeLine(int count) {
        for (int i = 0; i < count && op.opModeIsActive(); i++) {
            nudgeIntake(BioBuzzConstants.PILE_SPACING_S);
            op.waitSeconds(BioBuzzConstants.INTAKE_SETTLE_S);
            op.ballCount++;
        }
    }

    /** Task 2b: pile — short then longer nudges. */
    public void intakePile() {
        for (int i = 0; i < BioBuzzConstants.AUTO_MAX_POLLEN && op.opModeIsActive(); i++) {
            double seconds = (i < 2) ? BioBuzzConstants.NUDGE_SHORT_S : BioBuzzConstants.NUDGE_LONG_S;
            nudgeIntake(seconds);
            op.waitSeconds(BioBuzzConstants.INTAKE_SETTLE_S);
            op.ballCount++;
        }
    }

    /** Task 3a: wall — approach, strafe along, then pull off. */
    public void intakeOffWall() {
        nudgeIntake(BioBuzzConstants.WALL_APPROACH_S);
        op.strafeBy(BioBuzzConstants.CORNER_STRAFE_S, BioBuzzConstants.DRIVE_STRAFE_POWER);
        nudgeIntake(BioBuzzConstants.NUDGE_SHORT_S);
        op.ballCount++;
    }

    /** Task 3b: corner — two-axis timed recovery. */
    public void intakeFromCorner() {
        nudgeIntake(BioBuzzConstants.NUDGE_SHORT_S);
        op.strafeBy(BioBuzzConstants.CORNER_STRAFE_S, -BioBuzzConstants.DRIVE_STRAFE_POWER);
        nudgeIntake(BioBuzzConstants.NUDGE_SHORT_S);
        op.ballCount++;
    }

    /** Timed waypoints instead of splines — no localizer on this config. */
    public void navigateAndIntake(double... segmentSeconds) {
        for (double seconds : segmentSeconds) {
            if (!op.opModeIsActive()) break;
            nudgeIntake(seconds);
        }
    }

    public void shoot(int balls) {
        op.shoot(balls);
    }
}
