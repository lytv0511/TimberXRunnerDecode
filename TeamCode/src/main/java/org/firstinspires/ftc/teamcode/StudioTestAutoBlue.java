package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoBase;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzConstants;

/**
 * Blue autonomous — LEGACY mirror of Red. Use BioBuzzSearchAuto for real vision search.
 * Toggle USE_VISION_SEARCH to test vision on same opmode.
 */
@Config
@Autonomous(name = "StudioTestAutoBlue", group = "Autonomous")
public class StudioTestAutoBlue extends BioBuzzAutoBase {

    public static boolean USE_VISION_SEARCH = false;
    public static int VISION_TARGET = 3;

    private double augPos1, augPos2, augPos3;

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(0, 0, 0);
    }

    @Override
    protected double getAllianceSign() {
        return BioBuzzConstants.BLUE_SIGN; // -1.0 mirrors heading
    }

    @Override
    protected void runAuto() {
        if (USE_VISION_SEARCH) {
            Pose2d cur = drive.localizer.getPose();
            com.acmerobotics.roadrunner.ftc.Actions.runBlocking(
                    drive.actionBuilder(cur).lineToX(cur.position.x - 3.5).build());
            int got = runSearchAndIntake(VISION_TARGET, 24.0);
            telemetry.addData("Vision got", got);
            telemetry.update();
            waitSeconds(0.5);
            intakeOff();
            return;
        }

        augPos1 = BioBuzzConstants.TICKS_PER_REV / 2.0;
        augPos2 = BioBuzzConstants.TICKS_PER_REV * 5.0 / 6.0;
        augPos3 = BioBuzzConstants.TICKS_PER_REV / 6.0;

        driveLineToX(-51);
        turnBy(10); // sign flips to -10 for Blue

        launchSequence();

        turnBy(-85); // flips to +85
        strafeBy(-10);
        driveLineToX(7);

        pollen.intakePile();

        driveLineToX(-11.5);
        strafeBy(10);
        turnBy(85); // flips to -85

        launchSequence();
        intakeOff();
    }

    private void launchSequence() {
        busy = true;
        launcherFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherFlywheel.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));
        launcherFlywheel.setVelocity(BioBuzzConstants.LAUNCHER_TARGET_VEL);
        sleep((long) BioBuzzConstants.LAUNCHER_SPINUP_MS);

        double[] augPositions = {augPos1, augPos2, augPos3};
        for (double augPos : augPositions) {
            if (!opModeIsActive()) break;
            ElapsedTime t = new ElapsedTime();
            t.reset();
            while (opModeIsActive()
                    && Math.abs(launcherFlywheel.getVelocity() - BioBuzzConstants.LAUNCHER_TARGET_VEL)
                    > BioBuzzConstants.LAUNCHER_VEL_TOL
                    && t.seconds() < 1.0) idle();

            sorterTo(augPos);
            ElapsedTime a = new ElapsedTime();
            a.reset();
            while (opModeIsActive()
                    && Math.abs(sorter.getCurrentPosition() - augPos) > 20
                    && a.seconds() < 0.5) idle();

            launcherElevator.setPower(BioBuzzConstants.ELEVATOR_FEED_POWER);
            waitSeconds(BioBuzzConstants.AUTO_FEED_S);
            launcherElevator.setPower(0);
        }
        sorterTo(BioBuzzConstants.SORTER_POS_0);
        launcherFlywheel.setPower(0);
        launcherElevator.setPower(0);
        busy = false;
        ballCount = 0;
    }
}
