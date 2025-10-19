package org.firstinspires.ftc.teamcode.util.opencv;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagLocalizationAndMotif extends OpMode {
    // PRob make a pid on bearing(angle) for turret
    //CHECK DOCS IF U DONT REMEMBER, ME!!!!!
    public enum motifPatterns {
        PPG,
        PGP,
        GPP
    }
    motifPatterns motifPattern;

    //Motif
    Position Cam1Pos = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    YawPitchRollAngles cam1Orientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    AprilTagProcessor tag1;
    VisionPortal visionPortal1;

    //TurretTrackerfing

    AprilTagProcessor tag2;
    //Prob gonna add a color tracker for this but too lazy rn to find the documentation


    VisionPortal visionPortal2;



    @Override
    public void init() {
        //Motif
        tag1 = new AprilTagProcessor.Builder()
                .setCameraPose(Cam1Pos,cam1Orientation)
                .build();

        VisionPortal.Builder PortalTag1 = new VisionPortal.Builder();


        PortalTag1.setCamera(hardwareMap.get(WebcamName.class,"Webcam1"));

        PortalTag1.addProcessor(tag1);


        //Turret
        tag2 = new AprilTagProcessor.Builder()
                .build();

        VisionPortal.Builder PortalTag2 = new VisionPortal.Builder();

        PortalTag2.setCamera(hardwareMap.get(WebcamName.class,"Webcam2"));

        PortalTag2.addProcessor(tag2);


        visionPortal1 = PortalTag1.build();
        visionPortal2 = PortalTag2.build();
        List<AprilTagDetection> motif = tag1.getDetections();

        for(AprilTagDetection detection : motif){
            if (detection.id == 21){
                motifPattern = motifPatterns.GPP;
            }else if(detection.id == 22){
                motifPattern = motifPatterns.PGP;
            }else if(detection.id == 23){
                motifPattern = motifPatterns.PPG;
            }else{
                telemetry.addLine("MOTIF not detected");
            }
        }






    }


    @Override
    public void loop() {
        //Tag1 Telemetry
        List<AprilTagDetection> currentDetections = tag2.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detections : currentDetections) {
            if (detections.metadata != null){
                telemetry.addLine(String.format("\n==== (ID %d) %s", detections.id, detections.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detections.robotPose.getPosition().x,
                        detections.robotPose.getPosition().y,
                        detections.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detections.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detections.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detections.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detections.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detections.center.x, detections.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }
}
