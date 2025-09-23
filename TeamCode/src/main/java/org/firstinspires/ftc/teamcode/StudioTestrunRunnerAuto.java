package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="StudioTestrunRunnerAuto")
public class StudioTestrunRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(27) // forward 27
                        .turn(Math.toRadians(-90)) // turn 90° clockwise
                        .lineToY(-48) // forward 48 in new heading
                        .turn(Math.toRadians(90)) // turn back to original heading
                        .lineToX(72) // forward 72 along original heading
                        .turn(Math.toRadians(-45))
                        .splineTo(new Vector2d(0, -20), Math.toRadians(0))
                        .build());
    }
}
