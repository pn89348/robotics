package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.openCV.eocv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "ColorDetectionTestRed")
public class openCvColorDetection extends OpMode {
    OpenCvWebcam webcam1 = null;


    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam1, 0);
        webcam1.setPipeline(new ColorDetectionPipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {

    }

    class ColorDetectionPipeline extends OpenCvPipeline {
        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        Mat YCbCr = new Mat();
        Mat RedCrop;
        Mat BlueCrop;

        double Redavgfin;
        double Blueavgfin;
        Mat outPut = new Mat();
        Scalar RectColor = new Scalar(255, 0, 0, 0);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline is running");
            Rect Fullrect = new Rect(1, 1, 639, 359);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, Fullrect, RectColor, 2);

            RedCrop = YCbCr.submat(Fullrect);
            BlueCrop = YCbCr.submat(Fullrect);

            Core.extractChannel(RedCrop, RedCrop, 2);
            Core.extractChannel(BlueCrop, BlueCrop, 1);
            Scalar redavg = Core.mean(RedCrop);
            Scalar blueavg = Core.mean(RedCrop);
            Redavgfin =  redavg.val[0];
            Blueavgfin =  blueavg.val[0];


            if (Redavgfin>99&& Redavgfin<108) {
                telemetry.addLine("Red sample has been detected");
                telemetry.addData("red value", Redavgfin);

            }
            else if (Redavgfin >=108 ) {
                telemetry.addLine("Yellow sample has been detected");
            }else if (Redavgfin < Blueavgfin){
                telemetry.addLine("blue sample has been detected");
            }else{
                telemetry.addLine("No samples detected");
            }
            telemetry.update();
            return outPut;
        }
    }
}