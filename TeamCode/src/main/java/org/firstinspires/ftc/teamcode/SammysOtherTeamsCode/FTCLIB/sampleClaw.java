package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.FTCLIB;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class sampleClaw extends SubsystemBase {
    Servo sampleClaw;
    public sampleClaw(HardwareMap hardwareMap, String name){
sampleClaw = hardwareMap.get(Servo.class,name);
    }

    public void openClaw(){
        sampleClaw.setPosition(0.15);
    }
    public void closeClaw(){
        sampleClaw.setPosition(0.4);
    }
}
