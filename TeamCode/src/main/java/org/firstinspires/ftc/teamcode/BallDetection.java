package org.firstinspires.ftc.teamcode;


import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;


import java.util.List;
@TeleOp(name = "Ball Detection")
public class BallDetection extends LinearOpMode
{
    public static final double objectWidthInRealWorldUnits = 5;  // Replace with the actual width of the object in real-world units
    public static final double width = 60;
    public static final double focalLength = 360;
    double leftZone = width / 3; // ~106 px
    double rightZone = 2 * width / 3; // ~213 px

    private static double getDistance( double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    @Override
    public void runOpMode()
    {
        ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN) // <--- SET BALL COLOR HERE
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())       // search entire camera view
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // <--- SET BALL COLOR HERE
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())       // search entire camera view
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        // Building the  VisionPortal
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(greenLocator)
                .addProcessor(purpleLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.addLine("Initialized. Press Play to start detecting balls.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
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
                double x = box.center.x;
                double pixelWidthP = Math.max(box.size.width, box.size.height);
                if (x<= rightZone)
                    telemetry.addData("Purple",
                            "Distance: %.2f  Y: %.2f  Area: %d",
                            (getDistance(pixelWidthP) / 2.0) * (5.0 / 3.0),
                            getDistance(pixelWidthP),
                            p.getContourArea());
            }

            for (ColorBlobLocatorProcessor.Blob g : greenBlobs)
            {
                RotatedRect box = g.getBoxFit();
                double pixelWidthG = Math.max(box.size.width, box.size.height);


                telemetry.addData("Green",
                        "Distance: %.2f  Y: %.2f  Area: %d",
                        (getDistance(pixelWidthG) / 2.0) * (5.0 / 3.0),
                        getDistance(pixelWidthG),
                        g.getContourArea());
            }

            telemetry.update();
            sleep(50);
        }
    }
}


