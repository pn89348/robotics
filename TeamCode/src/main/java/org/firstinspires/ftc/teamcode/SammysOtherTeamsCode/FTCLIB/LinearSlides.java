package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.FTCLIB;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;


public class LinearSlides extends SubsystemBase {

    private PIDController controller;

    public static double p =0.02,i=0, d =0;
    public static double f=0.15;





    DcMotor lsl; // linear slides left
    DcMotor lsr; // linear slides right

public LinearSlides(HardwareMap hardwareMap,String namelsl,String namelsr){
    lsl = hardwareMap.get(DcMotor.class,namelsl);
    lsr = hardwareMap.get(DcMotor.class,namelsr);
    lsr.setDirection(DcMotorSimple.Direction.REVERSE);
    controller = new PIDController(p, i, d);

}










      public void PIDloop(int Target) {
          controller.setPID(p, i, d);
          //  double BothLSCurrentPos = lsl.getCurrentPosition()*0.5 + lsr.getCurrentPosition()*0.5;
//            double lsAveragePos = BothLSCurrentPos;

          double lsAveragePos = lsl.getCurrentPosition();
          /* the method for this is that the position for lsl and lsr is generally the the same. I have included a
          commented out code which gets both, I just didnt want it to be too complex
           */
          double pid = controller.calculate(lsAveragePos, Target);
          double ff = f;
          double power = pid + ff;

          lsl.setPower(power);
          lsr.setPower(power);
   }

   public void setManualPower(double speed){
    lsl.setPower(speed);
    lsr.setPower(speed);
   }

   public void runToPosition(int targetPos, double speed){
    lsl.setTargetPosition(targetPos);
    lsr.setTargetPosition(targetPos);
    lsr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lsl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lsl.setPower(speed);
    lsl.setPower(speed);
    }

}






