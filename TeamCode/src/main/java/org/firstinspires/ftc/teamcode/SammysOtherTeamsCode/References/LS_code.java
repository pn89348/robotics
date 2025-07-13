package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "LS_code", group = "ETAT")
public class LS_code extends LinearOpMode {
    //Hardware Map Names
    String LeftLinearSlideName = "lsl", RightLinearSlideName = "lsr";

    // For linear slides
    public DcMotor LeftLinearSlide = null;
    public DcMotor RightLinearSlide = null;

    final double LS_TICKS_PER_MM = 537.7 / 120.0;

    double LS_full_speed = 0.8;   //0.8 or 1.0
    double LS_half_speed = LS_full_speed/3;   // div 2
    int LS_TargetPosition = (int)(0 * LS_TICKS_PER_MM);
    boolean LS_motorRunning = false;
    double LS_RESET_POSITION = 0;
    double LS_TOLERANCE = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Telemetry.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Robot Hardware Mapping:
        LeftLinearSlide = hardwareMap.dcMotor.get(LeftLinearSlideName);
        RightLinearSlide = hardwareMap.dcMotor.get(RightLinearSlideName);

        // Robot Hardware Configuration:
        //For Linear SLide
        RightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initial conditions:
        //For Linear SLide
        LS_motorRunning = false;
        int LS_value = 0;
        LS_TargetPosition = (int) (LS_value * LS_TICKS_PER_MM);
        LeftLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
        RightLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Initialization & Configuration : Press START > button ");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // For Linear Slide Code start==========================================================
            if ((gamepad2.dpad_down) && (!LS_motorRunning)){
                LS_value = 0;
                LS_TargetPosition = (int)(LS_value * LS_TICKS_PER_MM);
                LeftLinearSlide.setTargetPosition((int)(LS_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int)(LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
                while(LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()){idle();}
                LS_motorRunning = true;
            }

            if ((gamepad2.dpad_up) && (!LS_motorRunning)){
                LS_value = 483;
                LS_TargetPosition = ((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
                while(LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()){idle();}
                LS_motorRunning = true;
            }

            if ((gamepad2.dpad_right) && (!LS_motorRunning)){
                LS_value = 50;
                LS_TargetPosition = ((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
                while(LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()){idle();}
                LS_motorRunning = true;
            }

            if (Math.abs(LeftLinearSlide.getCurrentPosition() - LS_TargetPosition) < LS_TOLERANCE && LS_TargetPosition!= LS_RESET_POSITION) {
                LeftLinearSlide.setPower(LS_half_speed);
                RightLinearSlide.setPower(LS_half_speed);
                LS_motorRunning=false;
            }
            if (Math.abs(RightLinearSlide.getCurrentPosition() - LS_TargetPosition) < LS_TOLERANCE && LS_TargetPosition!= LS_RESET_POSITION) {
                LeftLinearSlide.setPower(LS_half_speed);
                RightLinearSlide.setPower(LS_half_speed);
                LS_motorRunning=false;
            }

            if((LeftLinearSlide.getCurrentPosition()>=LS_RESET_POSITION && LeftLinearSlide.getCurrentPosition()<=10) && LS_TargetPosition==LS_RESET_POSITION ) {
                LeftLinearSlide.setPower(0);
                RightLinearSlide.setPower(0);
                LS_motorRunning=false;
            }
            if((RightLinearSlide.getCurrentPosition()>=LS_RESET_POSITION && RightLinearSlide.getCurrentPosition()<=10) && LS_TargetPosition==LS_RESET_POSITION ) {
                LeftLinearSlide.setPower(0);
                RightLinearSlide.setPower(0);
                LS_motorRunning=false;
            }
            // For Linear Slide Code end

        }
    }
}