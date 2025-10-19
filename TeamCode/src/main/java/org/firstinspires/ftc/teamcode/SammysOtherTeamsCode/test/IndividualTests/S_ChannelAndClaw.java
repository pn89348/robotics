package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Config
@TeleOp(name = "IntakeTest")
public class S_ChannelAndClaw extends LinearOpMode {


    CRServo ChannelSlides;
    Servo claw;
    Servo clawPivot;
    double ClawPivotPos = 0;
    double ClawPivotPos2;
    public static boolean right_bumper;
    public static boolean x;
    boolean ClawClosed = false;
    boolean ChannelSLidesExtended = false;
    public static boolean y;
    public static boolean b;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        ChannelSlides = hardwareMap.get(CRServo.class, "channelSlides");
        clawPivot.setPosition(0.5);


        right_bumper = gamepad2.right_bumper;

        claw = hardwareMap.get(Servo.class, "claw2");

        waitForStart();
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);


            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            if( right_bumper &&!right_bumper) {
                ClawClosed = !ClawClosed;
            }


            if (ClawClosed){
                claw.setPosition(0.3);//close
                sleep(1000);
            }else if (!ClawClosed){

                claw.setPosition(0);//open
                sleep(1000);
            }


            if (x && !ChannelSLidesExtended) {
                ChannelSlides.setPower(1);
                sleep(1500);
                    ClawClosed = true;
                    ChannelSlides.setPower(0.1);
                }




            if(x && ChannelSLidesExtended){
                ChannelSlides.setPower(-1);
                sleep(1500);
                ClawClosed = true;
                ChannelSlides.setPower(-0.1);
                }
            if (y) {

              // if (ClawPivotPos>=0&&ClawPivotPos<=4) {
                   ClawPivotPos = ClawPivotPos + 1;
                   ClawPivotPos2 = ClawPivotPos / 8;
                   clawPivot.setPosition(ClawPivotPos2);
                   sleep(250);
//                while(PivotTime.time(TimeUnit.MILLISECONDS)<250){
//                    PivotTime.wait();
//                }
//             //  }else if(ClawPivotPos>4){
//                   ClawPivotPos = 4;
//                   ClawPivotPos2 = ClawPivotPos/8;
//             //  }else if(ClawPivotPos<0){
//                   ClawPivotPos = 0;
//                   ClawPivotPos2 = ClawPivotPos/8;
               }
            if (b) {

                //   if (ClawPivotPos >= 0 && ClawPivotPos <= 4) {
                ClawPivotPos = ClawPivotPos - 1;
                ClawPivotPos2 = ClawPivotPos / 8;
                clawPivot.setPosition(ClawPivotPos2);
                sleep(250);
//                while(PivotTime.time(TimeUnit.MILLISECONDS)<250){
//                    PivotTime.wait();
//                }

//               // } else if (ClawPivotPos > 4) {
//                    ClawPivotPos = 8;
//                    ClawPivotPos2 = ClawPivotPos / 8;
//               // } else if (ClawPivotPos < 0) {
//                    ClawPivotPos = 0;
//                    ClawPivotPos2 = ClawPivotPos / 8;
                //}
                // }

            }
        }
    }
}
