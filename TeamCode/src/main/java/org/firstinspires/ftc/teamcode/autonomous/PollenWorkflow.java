package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Planning-stage helpers for BIOBUZZ preview tasks:
 * - intake off floor
 * - intake multiple (line/pile)
 * - intake off wall / out of corner
 * - navigate + intake autonomously
 * No gamepad dependency — pure auto.
 */
public class PollenWorkflow {
    private final BioBuzzAutoBase op;
    private final MecanumDrive drive;
    private final DcMotor intake;
    private final DcMotor elevator;
    private final DcMotor sorter;

    public PollenWorkflow(BioBuzzAutoBase op, MecanumDrive drive,
                          DcMotor intake, DcMotor elevator, DcMotor sorter) {
        this.op = op;
        this.drive = drive;
        this.intake = intake;
        this.elevator = elevator;
        this.sorter = sorter;
    }

    // ----- Basic intake -----

    public void intakeOn() {
        intake.setPower(BioBuzzConstants.INTAKE_POWER);
        elevator.setPower(BioBuzzConstants.ELEVATOR_HOLD_POWER);
    }

    public void intakeOff() {
        intake.setPower(BioBuzzConstants.INTAKE_IDLE_POWER);
        elevator.setPower(0);
    }

    /** Nudge forward while intaking — precondition for wall/corner recovery. */
    public void nudgeIntake(double distanceIn) {
        Pose2d cur = drive.localizer.getPose();
        intakeOn();
        Actions.runBlocking(drive.actionBuilder(cur)
                .lineToX(cur.position.x + distanceIn)
                .build());
    }

    // ----- Preview tasks (tunable stubs) -----

    /** Task 1: single pollen off floor. */
    public void intakeSingle() {
        intakeOn();
        nudgeIntake(BioBuzzConstants.NUDGE_SHORT_IN);
        waitSec(0.4);
    }

    /** Task 2: multiple in a line — spaced by PILE_SPACING_IN. */
    public void intakeLine(int count) {
        intakeOn();
        for (int i = 0; i < count && op.opModeIsActive(); i++) {
            nudgeIntake(BioBuzzConstants.PILE_SPACING_IN);
            rotateSorterSlot();
            waitSec(0.2);
        }
    }

    /** Task 2b: pile — small nudges with sorter cycling. */
    public void intakePile() {
        intakeOn();
        for (int i = 0; i < BioBuzzConstants.AUTO_MAX_POLLEN && op.opModeIsActive(); i++) {
            double d = (i < 2) ? BioBuzzConstants.NUDGE_SHORT_IN : BioBuzzConstants.NUDGE_LONG_IN;
            nudgeIntake(d);
            rotateSorterSlot();
            waitSec(0.3);
        }
    }

    /** Task 3a: wall — approach, strafe along, then pull off. */
    public void intakeOffWall() {
        intakeOn();
        // Square to wall with a short forward
        nudgeIntake(BioBuzzConstants.WALL_APPROACH_IN * 0.5);
        // Pull along wall (Y)
        Pose2d cur = drive.localizer.getPose();
        Actions.runBlocking(drive.actionBuilder(cur)
                .strafeTo(new com.acmerobotics.roadrunner.Vector2d(cur.position.x,
                        cur.position.y + BioBuzzConstants.CORNER_STRAFE_IN * 0.5))
                .build());
        nudgeIntake(2.0);
    }

    /** Task 3b: corner — two-axis recovery, tuned after field CAD. */
    public void intakeFromCorner() {
        intakeOn();
        // Into corner
        nudgeIntake(2.0);
        // Strafe out while intaking
        Pose2d cur = drive.localizer.getPose();
        Actions.runBlocking(drive.actionBuilder(cur)
                .strafeTo(new com.acmerobotics.roadrunner.Vector2d(cur.position.x,
                        cur.position.y - 3.0))
                .build());
        nudgeIntake(1.5);
    }

    /** Task 4: navigate between known waypoints while intaking (road-runner). */
    public void navigateAndIntake(Pose2d... waypoints) {
        intakeOn();
        for (Pose2d wp : waypoints) {
            if (!op.opModeIsActive()) break;
            Pose2d cur = drive.localizer.getPose();
            Actions.runBlocking(drive.actionBuilder(cur)
                    .splineTo(wp.position, wp.heading.toDouble())
                    .build());
            waitSec(0.15); // let intake settle
        }
    }

    // ----- Sorter cycling (DECODE heritage — keep until sort/no-sort decided) -----

    private int slot = 0;
    private void rotateSorterSlot() {
        slot = (slot % 3) + 1;
        double target = (slot == 1) ? BioBuzzConstants.SORTER_POS_0
                : (slot == 2) ? BioBuzzConstants.SORTER_POS_1 : BioBuzzConstants.SORTER_POS_2;
        sorter.setTargetPosition((int) target);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(BioBuzzConstants.SORTER_POWER);
        while (op.opModeIsActive() && sorter.isBusy()) op.idle();
        sorter.setPower(0);
    }

    private void waitSec(double s) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (op.opModeIsActive() && t.seconds() < s) op.idle();
    }
}
