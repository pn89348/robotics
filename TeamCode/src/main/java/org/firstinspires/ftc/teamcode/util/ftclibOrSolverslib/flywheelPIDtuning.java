package org.firstinspires.ftc.teamcode.util.ftclibOrSolverslib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class flywheelPIDtuning extends OpMode {
//    InterpLUT dist =new InterpLUT(); the length doesnt  work on
//    april tags(all things involving distance will be commented out just in case i need it)
    InterpLUT vel = new InterpLUT();
    PIDFController Feedback;
    SimpleMotorFeedforward feedforward;
    PIDController teest;
    static double kP = 0;
    static double kI =0;
    static double kD = 0;
    static double kS = 0;
    static double kV = 0;
//    double length;// detected by camera
    double f = 0;// placholder cus in the loop it changes the f to the feedforward output
    DcMotorEx flyWheel;
    double range;

    AprilTagProcessor tag;
    VisionPortal visionPortal;
    @Override
    public void init() {
        tag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder PortalTag = new VisionPortal.Builder();

        PortalTag.setCamera(hardwareMap.get(WebcamName.class,"Webcam1"));

        PortalTag.addProcessor(tag);


        feedforward = new SimpleMotorFeedforward(kS,kV);
        Feedback = new PIDFController(kP,kI,kD,f);
        flyWheel = hardwareMap.get(DcMotorEx.class,"flywheel");
//        distLUTinit();
        velocityLUTinit();

        visionPortal = PortalTag.build();

    }

    @Override
    public void loop() {
        List<AprilTagDetection> currentdetections = tag.getDetections();
        for (AprilTagDetection detections:currentdetections){
            if (detections.metadata != null){
                if (detections.id == 24){
                    range = detections.ftcPose.range;
                } else if (detections.id == 20) {
                    range = detections.ftcPose.range;
                }
            }
        }

        Feedback.setPIDF(kP,kI,kD,f);
//        double distance = dist.get(length);// gonna use camera for this so ill update this later
        double velocity = vel.get(range);// gonna be based on distance or smt
        Feedback.setF(feedforward.calculate(flyWheel.getVelocity(),velocity));
        double power = Feedback.calculate(flyWheel.getVelocity(),velocity);



        flyWheel.setPower(power);

    }
//    public void distLUTinit(){
//        double pixelLength =0;//placeholders
//        double distance = 0;
//        dist.add(pixelLength,distance);// prob not gonna be a variable,
//        // and also have mutliple add statements
//        dist.createLUT();
//    }

    public void velocityLUTinit(){
        double distance = 0;// placeholders
        double velocity = 0;
        vel.add(distance,velocity);// same thing as above function
        vel.createLUT();

    }



}
