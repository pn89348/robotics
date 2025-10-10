package org.firstinspires.ftc.teamcode.TeleOp.Archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.controller.PIDController;

@Disabled
@Config
@TeleOp(name = "PID")
public class PIDTuning extends LinearOpMode {


    private PIDController controller;

    public static double p =0,i=0, d =0;
    public static double f=0;


    public static int target;

    DcMotor lsl; // linear slides left
    DcMotor lsr; // linear slides right


    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lsl = hardwareMap.get(DcMotor.class,"lift_left");
        lsr = hardwareMap.get(DcMotor.class,"lift_right");

        lsr.setDirection(DcMotorSimple.Direction.REVERSE);
        lsr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsl.setDirection(DcMotorSimple.Direction.REVERSE);
        lsr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        waitForStart();
        while (opModeIsActive()) {
            controller.setPID(p,i,d);

            //  double BothLSCurrentPos = lsl.getCurrentPosition()*0.5 + lsr.getCurrentPosition()*0.5;
//            double lsAveragePos = BothLSCurrentPos;

            double lsAveragePos = lsl.getCurrentPosition();// the method for this is that the position for lsl and lsr is
            //generally the the same. I have included a commented out code wich gets both, I just didnt want it to be too complex;
            double pid = controller.calculate(lsAveragePos, target);
            double ff =f;
            double power = pid + ff;

            lsl.setPower(power);
            lsr.setPower(power);

            telemetry.addData("target", target);
            telemetry.addData("LS Pos", lsAveragePos);

            telemetry.update();
        }

    }
}

