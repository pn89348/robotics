package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
public class ColorSensing extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        PredominantColorProcessor proccessor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.entireFrame())
                .setSwatches(PredominantColorProcessor.Swatch.BLUE)
               // .setSwatches(PredominantColorProcessor.Swatch.RED)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(proccessor)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive() ||opModeInInit()){

            PredominantColorProcessor.Result result = proccessor.getAnalysis();


            telemetry.addData("Best Match:", result.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
            telemetry.update();
        }

    }
}
