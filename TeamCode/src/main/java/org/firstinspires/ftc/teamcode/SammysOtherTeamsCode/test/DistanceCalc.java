package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Config
public class DistanceCalc extends LinearOpMode {
    double width =1280;
    double height = 720;
    WebcamName weewooCam;
    public static double objectWidthInRealWorldUnits = 1.5;
    public static double focalLength = 1355.2;
    @Override
    public void runOpMode() throws InterruptedException {
        weewooCam = hardwareMap.get(WebcamName.class,"Webcam 1");
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setErodeSize(5)
                .setDilateSize(5)
                .setBlurSize(5)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size((int) width, (int) height))
                .setCamera(weewooCam)
                .build();
        waitForStart();
        while (opModeIsActive()) {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            if (blobs.isEmpty()) {

            }


            if (!blobs.isEmpty()) {
                ColorBlobLocatorProcessor.Util.filterByArea(200, 20000, blobs);
                ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
                ColorBlobLocatorProcessor.Blob LargestBlob = blobs.get(0);

                RotatedRect largestRect = LargestBlob.getBoxFit();

                double distance = (objectWidthInRealWorldUnits * focalLength) / largestRect.size.width;
            }
        }
    }
}
