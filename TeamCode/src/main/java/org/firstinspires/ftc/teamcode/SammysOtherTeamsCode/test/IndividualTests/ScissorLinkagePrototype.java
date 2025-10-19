package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
public class ScissorLinkagePrototype extends LinearOpMode {

  Servo Linkage;


    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        Linkage = hardwareMap.get(Servo.class,"szoneslide");



        waitForStart();


        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad2.x && !previousGamepad2.x){
            Linkage.setPosition(Linkage.getPosition()+0.1);
        }
        if (currentGamepad2.y && !previousGamepad2.y){
            Linkage.setPosition(Linkage.getPosition()-0.1);
        }
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
            Linkage.setPosition(Linkage.getPosition()+0.05);
        }
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
            Linkage.setPosition(Linkage.getPosition()-0.05);
        }

        telemetry.addData("Servo Position", Linkage.getPosition());
        telemetry.update();
    }
}
