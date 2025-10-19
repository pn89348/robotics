package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Config
@TeleOp(name = "ClawTest")
public class S_ClawTest extends LinearOpMode {
   Servo claw;
  public static boolean rightBumber ;
  public static boolean leftBumber;
    @Override
    public void runOpMode() throws InterruptedException {
      rightBumber= gamepad2.right_bumper;
      leftBumber = gamepad2.left_bumper;
        claw = hardwareMap.get(Servo.class,"claw");
        waitForStart();
       while(opModeIsActive()) {
           if (rightBumber) {
               claw.setPosition(0.3);//close
           }
           if (leftBumber) {
               claw.setPosition(0);//open
           }
       }
    }
}
