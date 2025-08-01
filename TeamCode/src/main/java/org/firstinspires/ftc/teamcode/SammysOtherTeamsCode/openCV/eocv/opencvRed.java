package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.openCV.eocv;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "OpenCVred")

public class opencvRed extends LinearOpMode {
    double cX = 0;
    double cY = 0;
   // Servo clawPivot;
    double width = 0;
    double angle = 0;
    public static boolean AngleFound;
    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 1.5;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1355.2;  // Replace with the focal length of the camera in pixels
    double WP1 = 0;
    double WP2 = 0.1625;
    double WP3 = 0.325;
    double WP4 = 0.4875;
    double WP5 = 0.65;



    @Override
    public void runOpMode() {
       // clawPivot = hardwareMap.get(Servo.class, "wrist");
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
//
//            if (!Double.isNaN(getAngle())) { // we only update the angle if its not a null value
//                angle = getAngle();
//                telemetry.addData("Angle status", "updating");
//            } else {
//                telemetry.addData("Angle status", "not updating");
//            }


//                double oneDegree = 0.00278;
//                double ClawPivotPos1 = (oneDegree * (int) angle);
//                double ClawPivotPos2 = 0;
//
//                if(ClawPivotPos1<=18){
//                    ClawPivotPos2 = WP1;
//                }else if (ClawPivotPos1<=0.36&&ClawPivotPos1 >0.18){
//                    ClawPivotPos2 = WP2;
//                }else if(ClawPivotPos1<=0.54&& ClawPivotPos1>0.36){
//                    ClawPivotPos2 = WP3;
//                }else if(ClawPivotPos1<=0.72&&ClawPivotPos1 >0.54){
//                    ClawPivotPos2 = WP4;
//                }else if (ClawPivotPos1<=0.90&&ClawPivotPos1>0.72){
//                    ClawPivotPos2 = WP5;
//                }


                telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
                telemetry.addData("Width",width);
                telemetry.addData("Distance in Inch", (getDistance(width)));
//                telemetry.addData("angle", ClawPivotPos1);
                telemetry.update();
//                if (gamepad2.a) {
//                    clawPivot.setPosition(ClawPivotPos2);
//                }
//                if (gamepad2.back){
//                    clawPivot.setPosition(WP3);
//                }




            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }


    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new RedBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }


    class RedBlobDetectionPipeline extends OpenCvPipeline {
        @Override

        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect Red regions
            Mat RedMask = preprocessFrame(input);

            // Find contours of the detected Red regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(RedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest Red contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 255), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 0.50, cY + 25), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour

                String angleLabel = "angle:" + getAngle(largestContour) + "degrees";
                Imgproc.putText(input, angleLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
               while(Double.isNaN(angle)&& !AngleFound) {
                   angle = getAngle(largestContour);
               }

                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();
                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
            // red samples
        Scalar lowerRed = new Scalar(110, 100, 100);
        Scalar upperRed = new Scalar (140, 255, 255);


            Mat RedMask = new Mat();

            Core.inRange(hsvFrame, lowerRed, upperRed, RedMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(RedMask, RedMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(RedMask, RedMask, Imgproc.MORPH_CLOSE, kernel);

            return RedMask;
        }





    }
    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }
    public static double getDistance(double width) {
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    public double getAngle(MatOfPoint largestContour) {


        if (largestContour == null || largestContour.toArray().length == 0) {
            // no countours were found so we return a null value
            return Double.NaN;
        }
        RotatedRect rotatedRect = Imgproc.minAreaRect(new  MatOfPoint2f(largestContour.toArray()));
        angle = rotatedRect.angle;

        return angle;

    }
}




