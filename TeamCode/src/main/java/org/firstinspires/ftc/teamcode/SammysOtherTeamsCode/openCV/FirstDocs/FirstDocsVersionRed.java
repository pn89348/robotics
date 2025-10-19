package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.openCV.FirstDocs;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "openCVredWithFTCJavaDocs")
public class FirstDocsVersionRed extends LinearOpMode {

    double angle;
    double WP1 = 0; // wrist/ pivot positions
    double WP2 = 0.1625;
    double WP3 = 0.325;

    Servo clawPivot;
    double ClawPivotPos1;
    double ClawPivotPos2;




    @Override
    public void runOpMode() throws InterruptedException {

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setErodeSize(5)
                .setDilateSize(5)
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);


        clawPivot = hardwareMap.get(Servo.class, "wrist");

        while (opModeIsActive() || opModeInInit()) {



            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
            ColorBlobLocatorProcessor.Blob largestBlob;
            if (blobs.size() == 0) {
                largestBlob = null;
            }
            else {
                largestBlob = blobs.get(0);
                RotatedRect boxFit = largestBlob.getBoxFit();

                org.opencv.core.Size  myBoxFitSize;
                      myBoxFitSize  = boxFit.size;
                      
                 double width;
                 double height;
                      if (boxFit.angle == 90){
                           height = myBoxFitSize.width;
                           width = myBoxFitSize.height;
                 }else{
                          width = myBoxFitSize.width;
                          height = myBoxFitSize.height;
                      }
                telemetry.addData("width", width);
                telemetry.addData("height", height);
                telemetry.addData("angle",boxFit.angle);


                    angle = (int) boxFit.angle;





            }

            ClawPivotPos1 = 0.00278*angle;

            if(ClawPivotPos1<=0.30){
                ClawPivotPos2 = WP1;
            }else if (ClawPivotPos1<=0.60&&ClawPivotPos1 >0.30){
                ClawPivotPos2 = WP2;
            }else if(ClawPivotPos1<=0.90&& ClawPivotPos1>0.60) {
                ClawPivotPos2 = WP3;
            }
            if (gamepad2.a) {
                clawPivot.setPosition(ClawPivotPos2);
            }
            if (gamepad2.back){
                clawPivot.setPosition(WP3);
            }


            telemetry.update();


        }
    }
}
