package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.FTCLIB;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class submersibleSlides extends SubsystemBase {

    Servo subslides;
    public submersibleSlides(HardwareMap hardwareMap,String name){
    subslides = hardwareMap.get(Servo.class,name);
    }
    public void extend(){

        subslides.setPosition(0.48);
    }
    public void retract(){

        subslides.setPosition(-0.48);
    }

    public void extendhalf(){
        subslides.setPosition(0.15);
    }
}
