package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
public class NewArmTest extends OpMode {
Servo specClaw;
Servo specPivot;
Servo SpecElbow;
Servo SpecArmLeft;
Servo SpecArmRight;
Gamepad currentGamepad1 = new Gamepad();
Gamepad currentGamepad2 = new Gamepad();

Gamepad previousGamepad1 = new Gamepad();
Gamepad previousGamepad2 = new Gamepad();


//toggles
    boolean SpecClawOpen = true;

    boolean armInHangPosition = false;

    @Override
    public void init() {
        specClaw = hardwareMap.get(Servo.class,"specimenclaw");
        specPivot = hardwareMap.get(Servo.class,"specimenwrist");
        SpecArmLeft = hardwareMap.get(Servo.class,"specimenarmleft");
        SpecArmRight = hardwareMap.get(Servo.class,"specimenarmright");
        SpecElbow = hardwareMap.get(Servo.class, "specimenelbow");


        SpecArmLeft.setPosition(0.85);
        SpecArmRight.setPosition(0.85);
        SpecElbow.setPosition(0.53);
        specPivot.setPosition(0.025);
        specClaw.setPosition(0.25);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        //SpecClaw code start
        if(currentGamepad2.right_bumper&&!previousGamepad2.left_bumper){
            SpecClawOpen = !SpecClawOpen;
        }
        if(SpecClawOpen){
            specClaw.setPosition(0.25);
        }else{
            specClaw.setPosition(0);
        }
        //SpecClaw code end

        //SpecClaw arm+elbow+wrist

        if(currentGamepad2.y&&!previousGamepad2.y){
            armInHangPosition = !armInHangPosition;
        }

        if(armInHangPosition){
               specPivot.setPosition(0.78);
              SpecElbow.setPosition(0.09);
              SpecArmLeft.setPosition(0.05);
              SpecArmRight.setPosition(0.05);
        }else{

            SpecArmLeft.setPosition(0.85);
            SpecArmRight.setPosition(0.85);
            SpecElbow.setPosition(0.53);
            specPivot.setPosition(0.025);
            specClaw.setPosition(0.25);
        }

    }
}
