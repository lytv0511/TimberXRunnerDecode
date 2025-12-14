package org.firstinspires.ftc.teamcode;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@Autonomous(name="StudioConceptAuto", group = "Autonomous")
public class StudioRunnerConceptAuto extends LinearOpMode {
    private StudioAprilTag studioAprilTag;
    private ColorBlobLocatorProcessor greenLocator;
    private ColorBlobLocatorProcessor purpleLocator;
    // Hardware declarations
    private com.qualcomm.robotcore.hardware.DcMotorEx launcherFlywheel;
    private com.qualcomm.robotcore.hardware.DcMotor launcherElevator;
    private com.qualcomm.robotcore.hardware.DcMotor sorter;
    private com.qualcomm.robotcore.hardware.CRServo intakeServo;

    private MecanumDrive drive;

    private double ticksPerRevolution = 537.7;
    private double augPos1, augPos2, augPos3;

    boolean ball = false;

//    private ColorBlobLocatorProcessor greenLocator;
//    private ColorBlobLocatorProcessor purpleLocator;

    public static final double objectWidthInRealWorldUnits = 5;
    public static final double width = 60;
    public static final double focalLength = 360;
    double leftZone = width / 3; // ~106 px
    double rightZone = 2 * width / 3; // ~213 px
    int zone;  // 0- left,  1 - center,   2- right,  -1 - not found
    double extraDistance = 5;
    double x;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        studioAprilTag = new StudioAprilTag();
        studioAprilTag.init(hardwareMap, "Webcam 1");

        greenLocator = studioAprilTag.getGreenProcessor();
        purpleLocator = studioAprilTag.getPurpleProcessor();

        // Hardware initialization
        launcherFlywheel = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "launcherFlywheel");
        launcherElevator = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "launcherElevator");
        sorter = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "sorter");
        intakeServo = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "intakeServo");

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE);

        augPos1 = ticksPerRevolution / 2;
        augPos2 = (ticksPerRevolution * 5) / 6;
        augPos3 = ticksPerRevolution / 6;


//        ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.ARTIFACT_GREEN) // <--- SET BALL COLOR HERE
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .setRoi(ImageRegion.entireFrame())       // search entire camera view
//                .setDrawContours(true)
//                .setBlurSize(5)
//                .build();
//
//        ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // <--- SET BALL COLOR HERE
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .setRoi(ImageRegion.entireFrame())       // search entire camera view
//                .setDrawContours(true)
//                .setBlurSize(5)
//                .build();
//
//        // Building the  VisionPortal
//        VisionPortal portal = new VisionPortal.Builder()
//                .addProcessor(greenLocator)
//                .addProcessor(purpleLocator)
//                .setCameraResolution(new Size(320, 240))
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();

        telemetry.addLine("Initialized. Press Play to start detecting balls.");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        int tagId;
        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = studioAprilTag.getDetections();
            studioAprilTag.updatePose();

            if (!detections.isEmpty()) {
                tagId = detections.get(0).id;
                telemetry.addData("Detected Tag ID", tagId);

                Position pos = studioAprilTag.getRobotPosition();
                YawPitchRollAngles ori = studioAprilTag.getRobotOrientation();

                if (pos != null && ori != null) {
                    telemetry.addData("Robot X", "%.2f", pos.x);
                    telemetry.addData("Robot Y", "%.2f", pos.y);
                    telemetry.addData("Robot Z", "%.2f", pos.z);
                    telemetry.addData("Robot Yaw", "%.2f", ori.getYaw(AngleUnit.DEGREES));
                    telemetry.addData("Robot Pitch", "%.2f", ori.getPitch(AngleUnit.DEGREES));
                    telemetry.addData("Robot Roll", "%.2f", ori.getRoll(AngleUnit.DEGREES));
                }

            } else {
                telemetry.addLine("No tag detected");
            }
            telemetry.update();
            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .setTangent(Math.toRadians(0))   // heading right (x-positive)
                        .strafeTo(new Vector2d(0, 10))  // go left/right based on field coords
                        .setTangent(Math.toRadians(180)) // now we want to go backwards toward -X
                        .lineToX(-50)                    // RR now has a valid heading
                        .build()
        );

        sleep(500);

//        defaultLaunchSequence();

        sleep(5000);

        Actions.runBlocking(
                drive.actionBuilder( new Pose2d(0, 0, 0))
                        .setTangent(Math.toRadians(0))
                        .strafeTo(new Vector2d(0, 20))
                        .lineToY(10)
                        .build()
        );


        while (opModeIsActive()){
            DetectBalls();

        }




// **************************************************************************************************************
//*********************************************************************************************************************
//        tagId = detectTagID();
//        telemetry.addData("Detected Tag ID after scanning", tagId);
//
//        String tagName = studioAprilTag.getTagName();
//        if (tagName != null) {
//            telemetry.addData("Tag Name", tagName);
//        }
//
//        Position pos = studioAprilTag.getRobotPosition();
//        YawPitchRollAngles ori = studioAprilTag.getRobotOrientation();
//        if (pos != null && ori != null) {
//            telemetry.addData("Robot X", "%.2f", pos.x);
//            telemetry.addData("Robot Y", "%.2f", pos.y);
//            telemetry.addData("Robot Z", "%.2f", pos.z);
//            telemetry.addData("Robot Yaw", "%.2f", ori.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Robot Pitch", "%.2f", ori.getPitch(AngleUnit.DEGREES));
//            telemetry.addData("Robot Roll", "%.2f", ori.getRoll(AngleUnit.DEGREES));
//        }
//        telemetry.update();
//
//        Action trajectoryActionChosen;
//        Pose2d currentPose = drive.localizer.getPose();
//        if (tagId == 21) {
//            trajectoryActionChosen = drive.actionBuilder(currentPose)
//                    .splineTo(new Vector2d(71, 0), Math.toRadians(-90))
//                    .lineToY(-48)
//                    .waitSeconds(0.5)
//                    .splineTo(new Vector2d(95 + 8, -40 + 8), Math.toRadians(-62))
//                    .build();
//        } else if (tagId == 22) {
//            trajectoryActionChosen = drive.actionBuilder(currentPose)
//                    .lineToX(36)
//                    .strafeTo(new Vector2d(36, 48))
//                    .build();
//        } else if (tagId == 23) {
//            trajectoryActionChosen = drive.actionBuilder(currentPose)
//                    .strafeTo(new Vector2d(46, 30))
//                    .build();
//        } else {
//            trajectoryActionChosen = drive.actionBuilder(currentPose)
//                    .waitSeconds(0.1)
//                    .build();
//        }
//
//        Actions.runBlocking(
//                drive.actionBuilder(currentPose)
//                        .lineToX(103)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(-32)
//                        .turn(Math.toRadians(28))
//                        .build()
//        );
//        sleep(5000);
//        Actions.runBlocking(trajectoryActionChosen);

        studioAprilTag.shutdown();
    }

    private int detectTagID() {
        List<AprilTagDetection> detections = studioAprilTag.getDetections();
        if (!detections.isEmpty()) return detections.get(0).id;
        return -1;
    }
// ********************  BALLS DETECTION ******************************************************
    private int getZone(double x){
        if (x>200){
            telemetry.addData("Zone", "RIGHT" );
            zone = 2;


        } else if (x<110) {
            telemetry.addData("Zone", "LEFT" );
            zone = 0;
        }else{
            zone = 1;
            telemetry.addData("Zone", "CENTER" );
        }
        return zone ;
    }

    private static double getDistance( double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    public void FollowBall(){

        while (getZone(x) !=0 && getZone(x) != 2){

            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                            .setTangent(Math.toRadians(0))   // heading right (x-positive)
                            .strafeTo(new Vector2d(0, 10))  // go left/right based on field coords
                            .setTangent(Math.toRadians(180)) // now we want to go backwards toward -X
                            .lineToX(-50)                    // RR now has a valid heading
                            .build()
            );
        }
    }
    void DetectBalls(){
        telemetry.update();
        ball = false;
        sleep(50);
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();

        // Remove tiny blobs (noise)
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                200, 20000, greenBlobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                200, 20000, purpleBlobs);

        for (ColorBlobLocatorProcessor.Blob p : purpleBlobs)
        {
            RotatedRect box = p.getBoxFit();
            getZone(box.center.x);
            double pixelWidthP = Math.max(box.size.width, box.size.height);
            ball = true;
            telemetry.addData("Purple",
                    "Distance: %.2f  Y: %.2f  Area: %d",
                    (getDistance(pixelWidthP) / 2.0) * (5.0 / 3.0)-2,
                    getDistance(pixelWidthP),
                    p.getContourArea());
        }

        for (ColorBlobLocatorProcessor.Blob g : greenBlobs)
        {
            RotatedRect box = g.getBoxFit();
            double pixelWidthG = Math.max(box.size.width, box.size.height);
            ball = true;
            getZone(box.center.x);
            telemetry.addData("Green",
                    "Distance: %.2f  Y: %.2f  Area: %d",
                    (getDistance(pixelWidthG) / 2.0) * (5.0 / 3.0)-2,
                    getDistance(pixelWidthG),
                    g.getContourArea());
        }

        telemetry.update();
        sleep(50);
    }

// ***************************************************************************************************

    // === Mirrored methods from TeleOp ===
    private boolean launcherSequenceBusy = false;

    private void initiateLaunchSequence(double targetAugPos1, double targetAugPos2, double targetAugPos3) {
        launcherSequenceBusy = true;
        launcherFlywheel.setPower(1.0);

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos1);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos2);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setTargetPosition((int) targetAugPos3);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        launcherElevator.setPower(1.0);
        sleep(2000);
        launcherElevator.setPower(0);

        sorter.setTargetPosition(0);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.3);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }

        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        sorter.setPower(0);
        launcherElevator.setPower(0);
        launcherFlywheel.setPower(0);
        launcherSequenceBusy = false;
    }

    private void defaultIntakeSequence() {
        launcherSequenceBusy = true;
        sorter.setPower(0);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

        sorter.setTargetPosition(0);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }

        intakeServo.setPower(-1);
        sleep(2000);

        int pos2 = (int)(ticksPerRevolution / 3);
        sorter.setTargetPosition(pos2);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sleep(2000);

        int pos3 = (int)((ticksPerRevolution * 2) / 3);
        sorter.setTargetPosition(pos3);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sleep(2000);

        sorter.setTargetPosition(0);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.5);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sleep(2000);

        sorter.setPower(0);
        intakeServo.setPower(0);
        sorter.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        launcherSequenceBusy = false;
    }
    private void defaultLaunchSequence() {
        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherSequenceBusy = true;
        launcherFlywheel.setVelocity(6000 * 537.7);
        launcherElevator.setPower(-1);
        sleep(2000);
        sorter.setPower(0);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);

        // Step 1: augPos3
        sorter.setTargetPosition((int)augPos3);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sleep(1200);


        // Step 2: augPos1
        sorter.setTargetPosition((int)augPos1);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sleep(1550);


        // Step 3: augPos2
        sorter.setTargetPosition((int)augPos2);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }


        // Return to pos1
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(0.2);
        while (sorter.isBusy() && opModeIsActive()) { idle(); }
        sorter.setPower(0);
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherElevator.setPower(0);
        launcherFlywheel.setPower(0);
        launcherSequenceBusy = false;
    }
}
