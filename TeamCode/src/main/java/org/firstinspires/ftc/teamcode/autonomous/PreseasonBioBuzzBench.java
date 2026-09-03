package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Bench for BIOBUZZ Skill Builders — run on practice tiles with Pollen.
 * Select mode in Dashboard or edit before run. No field dependencies.
 * Hardware unchanged.
 */
@Config
@Autonomous(name = "PreseasonBioBuzzBench", group = "Preseason")
public class PreseasonBioBuzzBench extends BioBuzzAutoBase {

    public static int MODE = 0; // 0=floor hard-coded, 1=line, 2=pile, 3=wall, 4=corner, 5=nav waypoints, 6=VISION search (real)
    public static double WAYPOINT_X = 24;
    public static double WAYPOINT_Y = 12;
    public static int VISION_TARGET = 3;

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(0, 0, 0);
    }

    @Override
    protected double getAllianceSign() {
        return BioBuzzConstants.RED_SIGN;
    }

    @Override
    protected void runAuto() {
        telemetry.addData("Bench MODE", MODE);
        telemetry.update();

        switch (MODE) {
            case 0:
                pollen.intakeSingle();
                break;
            case 1:
                pollen.intakeLine(3);
                break;
            case 2:
                pollen.intakePile();
                break;
            case 3:
                pollen.intakeOffWall();
                break;
            case 4:
                pollen.intakeFromCorner();
                break;
            case 5:
                pollen.navigateAndIntake(
                        new Pose2d(12, 0, 0),
                        new Pose2d(WAYPOINT_X, WAYPOINT_Y, Math.toRadians(45)),
                        new Pose2d(36, 0, Math.toRadians(-30))
                );
                break;
            case 6:
                // Real vision search — use this to validate wall/corner priors
                int got = runSearchAndIntake(VISION_TARGET, 20.0);
                telemetry.addData("Vision got", got);
                telemetry.update();
                break;
            default:
                // Full cycle: floor -> wall -> corner
                pollen.intakeLine(2);
                driveLineToX(8);
                pollen.intakeOffWall();
                strafeBy(6);
                pollen.intakeFromCorner();
                break;
        }
        intakeOff();

        Pose2d p = drive.localizer.getPose();
        telemetry.addData("End X", "%.1f", p.position.x);
        telemetry.addData("End Y", "%.1f", p.position.y);
        telemetry.update();
        waitSeconds(1.0);
    }
}
