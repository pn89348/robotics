package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Config
@TeleOp(name = "ChannelSlidesTest",group = "GameCode")
public class S_ChannelSlidesTest extends LinearOpMode {
    CRServo ChannelSlides;


    boolean ChannelSLidesExtended = false;
    public static boolean x;
    @Override
    public void runOpMode() throws InterruptedException {

        ChannelSlides = hardwareMap.get(CRServo.class, "channelSlides");

        x= gamepad2.x;
        waitForStart();
        while (opModeIsActive()) {
            if(x){
                ChannelSLidesExtended = !ChannelSLidesExtended;
                if (!ChannelSLidesExtended) {
                    ChannelSlides.setPower(1);
                    sleep(750);
                    ChannelSlides.setPower(0.1);


                }
                else if (ChannelSLidesExtended) {
                    ChannelSlides.setPower(-1);
                    sleep(500);
                    ChannelSlides.setPower(-0.1);
                }
            }
        }
    }
}



