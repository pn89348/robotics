package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.FTCLIB;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class specClaw extends SubsystemBase{
Servo specClaw;
    public specClaw(HardwareMap hmap,String Name){
        specClaw =  hmap.get(Servo.class,Name);
    }
    public void openClaw(){
        specClaw.setPosition(0.3);//open
    }

    public void closeClaw(){
        specClaw.setPosition(0);// close
    }
    public void openClawBig(){
        specClaw.setPosition(0.35);
    }
}
