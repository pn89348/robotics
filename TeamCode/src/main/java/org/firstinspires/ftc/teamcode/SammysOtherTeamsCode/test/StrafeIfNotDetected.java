package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous(name = "StrafeIfNotDetected")
public class StrafeIfNotDetected extends LinearOpMode {
    DcMotor fl, fr, bl, br;
    Servo claw;
    MatOfPoint largestContour;


    boolean firstTime = true;

    boolean clawclosed = false;
    double cX = 0;
    double cY = 0;
    // Servo clawPivot;
    double width = 0;
    double angle = 0;
    private OpenCvCamera Cam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 1.5;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1355.2;  // Replace with the focal length of the camera in pixels

    @Override
    public void runOpMode()  throws InterruptedException {
        //clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(Cam, 30);

        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        while (opModeIsActive()){



        if(largestContour == null) {
            fl.setPower(0.25);
            fr.setPower(-0.25);
            bl.setPower(-0.25);
            bl.setPower(0.25);
        }
        }


        if(isStopRequested()) {
            // Release resources
            Cam.stopStreaming();
        }
    }


    private void initOpenCV() {

        RedBlobDetectionPipeline pipeline = new RedBlobDetectionPipeline();

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        Cam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        Cam.setPipeline(pipeline);

        Cam.openCameraDevice();
        Cam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }


    class RedBlobDetectionPipeline extends OpenCvPipeline {
       @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect Blue regions
            Mat BlueMask = preprocessFrame(input);





             // Find contours of the detected Blue regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(BlueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest Blue contour (blob)
            largestContour = findLargestContour(contours);




            if (largestContour != null) {
                if(firstTime){
                    fl.setPower(0.15);
                    fr.setPower(-0.15);
                    bl.setPower(-0.15);
                    bl.setPower(0.15);
                    sleep(500);
                    firstTime = false;
                }

                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                bl.setPower(0);

                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255,255), 2);

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

                angle = getAngle(largestContour);

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
            //blue samples
            Scalar lowerBlue = new Scalar(0, 100, 50);
            Scalar upperBlue = new Scalar(50, 255, 255);



            Mat BlueMask = new Mat();

            Core.inRange(hsvFrame, lowerBlue, upperBlue, BlueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(BlueMask, BlueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(BlueMask, BlueMask, Imgproc.MORPH_CLOSE, kernel);

            return BlueMask;
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
            // no countours were found so we return anull value
            return Double.NaN;
        }
        RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
        angle = rotatedRect.angle;
        if (rotatedRect.size.width < rotatedRect.size.height) {
            angle += 90;
        }

        return angle;
    }
}

