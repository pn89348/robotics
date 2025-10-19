package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test;

import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.SortOrder;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "weewoo")
public class StrafeINotDetectedv2 extends LinearOpMode{

    PIDController xController;
    PIDController yController;
    public static double xp =0,xi=0, xd =0;
    public static double yp =0,yi=0, yd =0;
    double width =1280;
    double height = 720;
    WebcamName weewooCam;
    DcMotor fl,fr,bl,br;
    double angle = 0;

    public static final double objectWidthInRealWorldUnits = 1.5;
    public static final double focalLength = 1355.2;


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
        xController = new PIDController(xp, xi, xd);
        yController = new PIDController(yp,yi,yd);


        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");
        waitForStart();
        while (opModeIsActive()) {

            xController.setPID(xp, xi, xd);
            xController.setPID(yp,yi,yd);


            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            if (blobs.isEmpty()) {

            }


            if (!blobs.isEmpty()) {
                ColorBlobLocatorProcessor.Util.filterByArea(200, 20000, blobs);
                ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
                ColorBlobLocatorProcessor.Blob LargestBlob = blobs.get(0);

                RotatedRect largestRect = LargestBlob.getBoxFit();

                double cx = largestRect.boundingRect().x + largestRect.size.width / 2.0;


                double frameCenterX = width / 2;


                double xOffsetPixels = cx - frameCenterX;

                double distance = (objectWidthInRealWorldUnits * focalLength) / largestRect.size.width;
                double y = yController.calculate(xOffsetPixels,0);
                double x = xController.calculate(distance,3.00);
                yController.setTolerance(40);
                xController.setTolerance(0.5);
                double x_rotated = x * Math.cos(angle) - y * Math.sin(angle);
                double y_rotated = x * Math.sin(angle) + y * Math.cos(angle);
                fl.setPower(x_rotated + y_rotated + 0);//i dont need theta rn i dont think
                bl.setPower(x_rotated - y_rotated + 0);
                fr.setPower(x_rotated - y_rotated - 0);
                br.setPower(x_rotated + y_rotated - 0);


            }
        }
    }
}

