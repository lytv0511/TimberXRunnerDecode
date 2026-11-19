package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

/**
 * Two-odometry-pod localizer using the goBILDA Pinpoint Odometry Computer.
 *
 * Hardware: one I2C device named "pinpoint" with two dead-wheel pods plugged
 * into it — the X (forward/parallel) pod and the Y (strafe/perpendicular) pod.
 * The Pinpoint's internal IMU supplies heading, so no separate "imu" device is
 * needed for odometry.
 */
@Config
public final class PinpointLocalizer implements Localizer {

    public static class Params {
        /** Use the built-in goBILDA 4-bar pod resolution (2000 CPR / 32mm wheel). */
        public boolean use4BarPods = true;
        /** Custom pod resolution in ticks per mm (used when use4BarPods = false).
         *  goBILDA swingarm pod = 13.26291192, 4-bar pod = 19.894. */
        public double customTicksPerMm = 19.894;

        /** Lateral offset of the X (forward) pod from the tracking point, in pod ticks.
         *  Left of center = positive. */
        public double parYTicks = 0.0;
        /** Longitudinal offset of the Y (strafe) pod from the tracking point, in pod ticks.
         *  Forward of center = positive. */
        public double perpXTicks = 0.0;

        public boolean reversePar = false;
        public boolean reversePerp = false;
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        // Config must contain a goBILDA Pinpoint Odometry Computer named "pinpoint".
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        final double ticksPerMm;
        if (PARAMS.use4BarPods) {
            driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            ticksPerMm = 19.894; // 2000 counts / (pi * 32mm)
        } else {
            ticksPerMm = PARAMS.customTicksPerMm > 0 ? PARAMS.customTicksPerMm : 19.894;
            driver.setEncoderResolution(ticksPerMm, DistanceUnit.MM);
        }

        // Pod offsets are configured in mm (first = X pod lateral offset,
        // second = Y pod longitudinal offset). Convert from pod ticks.
        double mmPerTick = 25.4 / ticksPerMm;
        driver.setOffsets(mmPerTick * PARAMS.parYTicks,
                mmPerTick * PARAMS.perpXTicks,
                DistanceUnit.MM);

        // X pod must count up when moving forward; Y pod when moving left.
        initialParDirection = PARAMS.reversePar
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = PARAMS.reversePerp
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD;
        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();
        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH),
                    driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
