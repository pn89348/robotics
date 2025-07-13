package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.openCV.pipelines;

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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RedBlobDetectionPipeline extends OpenCvPipeline {
    double cX = 0;
    double cY = 0;
    double angle;
    double width = 0;
    boolean AngleFound = false;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    MatOfPoint largestContour;

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




