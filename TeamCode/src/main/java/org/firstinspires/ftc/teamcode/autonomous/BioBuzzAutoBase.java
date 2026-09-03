package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.search.SearchAndIntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.search.SearchPlanner;
import org.firstinspires.ftc.teamcode.autonomous.search.ZoneSearchAction;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

/**
 * Planning-stage base for BIOBUZZ autos. Hardware mapping untouched.
 * Real autonomous: vision-driven search, not hard-coded paths.
 * SearchPlanner picks high-probability pollen cells; VisionApproachAction servos via PollenVision.
 */
@Config
public abstract class BioBuzzAutoBase extends LinearOpMode {

    // --- Hardware (same names as StudioTest autos) ---
    protected MecanumDrive drive;
    protected DcMotorEx launcherFlywheel;
    protected DcMotor launcherElevator;
    protected DcMotor sorter;
    protected DcMotor intake;

    protected PollenWorkflow pollen; // legacy helper (hard-coded nudges) — kept for bench tests
    protected PollenVision vision;
    protected SearchPlanner planner;

    // --- Alliance-aware start ---
    protected abstract Pose2d getStartPose();
    /** Alliance sign: 1.0 Red, -1.0 Blue — mirrors heading/strafe. */
    protected abstract double getAllianceSign();

    /** Implement your auto sequence here — keep it linear/readable. */
    protected abstract void runAuto();

    // --- Shared state ---
    protected int ballCount = 0;
    protected boolean busy = false;

    @Override
    public final void runOpMode() {
        initHardware(getStartPose());

        telemetry.addLine("BioBuzzAutoBase ready");
        telemetry.addData("Start", getStartPose());
        telemetry.addData("Alliance sign", getAllianceSign());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runAuto();

        // Safe stop
        if (drive != null) {
            Pose2d p = drive.localizer.getPose();
            telemetry.addLine("Auto complete");
            telemetry.addData("X", "%.1f", p.position.x);
            telemetry.addData("Y", "%.1f", p.position.y);
            telemetry.addData("Heading deg", "%.1f", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();
        }
        stopHardware();
    }

    // ----- Hardware lifecycle (do not rename hardware) -----

    protected void initHardware(Pose2d start) {
        drive = new MecanumDrive(hardwareMap, start);
        launcherFlywheel = hardwareMap.get(DcMotorEx.class, "launcherFlywheel");
        launcherElevator = hardwareMap.get(DcMotor.class, "launcherElevator");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        launcherElevator.setDirection(DcMotor.Direction.REVERSE);
        sorter.setDirection(DcMotor.Direction.REVERSE);

        // Flywheel closed-loop for consistent feed (DECODE heritage; keep until outtake decided)
        launcherFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));
        launcherFlywheel.setVelocity(BioBuzzConstants.LAUNCHER_TARGET_VEL);

        pollen = new PollenWorkflow(this, drive, intake, launcherElevator, sorter);

        // Vision + planner for real autonomous (no hard-coded paths)
        try {
            vision = new PollenVision();
            vision.init(hardwareMap, "Webcam 1"); // 640x480 — init may fail on RC without camera → catch
        } catch (Exception e) {
            telemetry.addData("Vision init failed", e.getMessage());
        }
        planner = new SearchPlanner();
        // Mirror Blue cells by flipping Y/heading if alliance sign <0
        if (getAllianceSign() < 0) mirrorPlannerForBlue();
    }

    private void mirrorPlannerForBlue() {
        // Flip Y and heading for Blue (Red is default). Rebuild list mirrored.
        java.util.List<org.firstinspires.ftc.teamcode.autonomous.search.SearchCell> mirrored = new java.util.ArrayList<>();
        for (org.firstinspires.ftc.teamcode.autonomous.search.SearchCell c : planner.getCells()) {
            double y = -c.pose.position.y;
            double h = Math.toRadians(180) - c.pose.heading.toDouble(); // rough mirror
            mirrored.add(new org.firstinspires.ftc.teamcode.autonomous.search.SearchCell(
                    c.name + "-B", new Pose2d(c.pose.position.x, y, h), c.prior));
        }
        planner.setCells(mirrored);
    }

    protected void stopHardware() {
        if (intake != null) intake.setPower(BioBuzzConstants.INTAKE_IDLE_POWER);
        if (launcherElevator != null) launcherElevator.setPower(0);
        if (launcherFlywheel != null) launcherFlywheel.setPower(0);
        if (sorter != null) sorter.setPower(0);
        if (vision != null) vision.close();
    }

    // ----- RoadRunner helpers (pose-relative, field-agnostic) -----

    protected void driveLineToX(double deltaIn) {
        Pose2d cur = drive.localizer.getPose();
        Actions.runBlocking(drive.actionBuilder(cur).lineToX(cur.position.x + deltaIn).build());
    }

    protected void driveLineToY(double deltaIn) {
        Pose2d cur = drive.localizer.getPose();
        Actions.runBlocking(drive.actionBuilder(cur).lineToY(cur.position.y + deltaIn).build());
    }

    protected void strafeBy(double deltaYIn) {
        Pose2d cur = drive.localizer.getPose();
        Actions.runBlocking(drive.actionBuilder(cur)
                .strafeTo(new Vector2d(cur.position.x, cur.position.y + deltaYIn))
                .build());
    }

    protected void turnBy(double deltaDeg) {
        double sign = getAllianceSign();
        Pose2d cur = drive.localizer.getPose();
        Actions.runBlocking(drive.actionBuilder(cur)
                .turn(Math.toRadians(deltaDeg * sign))
                .build());
    }

    protected void splineTo(Vector2d target, double headingDeg) {
        Pose2d cur = drive.localizer.getPose();
        Actions.runBlocking(drive.actionBuilder(cur)
                .splineTo(target, Math.toRadians(headingDeg * getAllianceSign()))
                .build());
    }

    protected void waitSeconds(double s) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (opModeIsActive() && t.seconds() < s) idle();
    }

    // ----- Intake/launch delegated to workflow -----

    protected void intakeOn() { pollen.intakeOn(); }
    protected void intakeOff() { pollen.intakeOff(); }

    // ----- Real autonomous: vision-driven search (no hard-coded paths) -----

    /** Run probability-based search until targetCount or timeout. Returns acquired. */
    protected int runSearchAndIntake(int targetCount, double timeoutSec) {
        if (vision == null || planner == null) {
            telemetry.addLine("Vision/planner missing — falling back to hard-coded");
            telemetry.update();
            return 0;
        }
        SearchAndIntakeAction search = new SearchAndIntakeAction(
                drive, vision, planner, this::intakeOn, this::intakeOff, targetCount, timeoutSec);
        Actions.runBlocking(search);
        return search.getAcquired();
    }

    /** Run zone-based search: drive to each zone, scan, approach, intake. Returns acquired. */
    protected int runZoneSearch(java.util.List<com.acmerobotics.roadrunner.Pose2d> zones,
                                int targetCount, double timeoutSec) {
        if (vision == null) {
            telemetry.addLine("Vision missing — cannot run zone search");
            telemetry.update();
            return 0;
        }
        ZoneSearchAction search = new ZoneSearchAction(
                drive, vision, zones, this::intakeOn, this::intakeOff, targetCount, timeoutSec);
        Actions.runBlocking(search);
        return search.getAcquired();
    }

    // ----- Sorter helpers (ticks) -----

    protected void sorterTo(double ticks) {
        sorter.setTargetPosition((int) ticks);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(BioBuzzConstants.SORTER_POWER);
        while (opModeIsActive() && sorter.isBusy()) idle();
        sorter.setPower(0);
    }
}
