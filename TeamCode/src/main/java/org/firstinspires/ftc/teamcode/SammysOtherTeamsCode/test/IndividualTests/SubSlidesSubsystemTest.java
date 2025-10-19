package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.FTCLIB.submersibleSlides;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "SubslidesTest")
public class SubSlidesSubsystemTest extends LinearOpMode {
    int ChannelSlidePos;
    boolean slidesRetracted;

    submersibleSlides channelSlides;

    @Override
    public void runOpMode() throws InterruptedException {
        channelSlides = new submersibleSlides(hardwareMap, "szoneslide-");
        ChannelSlidePos = 0;



        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if (currentGamepad2.x && !previousGamepad2.x) {

                if (ChannelSlidePos == 0) {
                    channelSlides.retract();
                    ChannelSlidePos =ChannelSlidePos +1;
                } else if (ChannelSlidePos == 1) {
                    channelSlides.extendhalf();
                    ChannelSlidePos =ChannelSlidePos +1;
                } else if (ChannelSlidePos == 2) {
                    channelSlides.extend();
                    ChannelSlidePos = 0;
                }
            }
        }
    }
}
