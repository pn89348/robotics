package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.FTCLIB;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class ClawPivot extends SubsystemBase {

    Servo ClawPivot;
    public  ClawPivot(HardwareMap hardwareMap, String name){
        ClawPivot = hardwareMap.get(Servo.class,name);
    }
    double ClawPivotPos;

    public void PivotGoingClockWise(){
        ClawPivotPos = ClawPivotPos - 0.1625;// goes in intervals of 0.1625
        ClawPivot.setPosition(ClawPivotPos);
    }

    public void PivotGoingCounterClockwise(){
        ClawPivotPos = ClawPivotPos + 0.1625;// goes in intervals of 0.1625
        ClawPivot.setPosition(ClawPivotPos);
    }
    public double GetClawPivot(){
        return ClawPivotPos;
    }

    public void setClawPivotManually(double ClawPivotPos){
        ClawPivot.setPosition(ClawPivotPos);
    }
 }

