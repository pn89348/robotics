package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Config
@TeleOp(name = "IntakeTestWithEnhancements(hopefully)")
public class S_ChannelAndClawAndDriveV2CusIdk extends LinearOpMode {


    CRServo ChannelSlides;
    Servo claw;
    Servo clawPivot;
    double ClawPivotPos = 3 ;
    double ClawPivotPos2;
    public static boolean rightBumper;
    public static boolean x;
    boolean ClawClosed = false;
    boolean ChannelSLidesExtended = false;

    public static boolean y;
    public static boolean b;
    //Hardware Map Names
    String flName = "fl", frName = "fr", blName = "bl", brName = "br";
    String LeftLinearSlideName = "lsl", RightLinearSlideName = "lsr";
    // Define or Declare Hardware here...
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor BackLeft = null;
    public DcMotor BackRight = null;

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
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        ChannelSlides = hardwareMap.get(CRServo.class, "channelSlides");
        clawPivot.setPosition(0.375);
        rightBumper= gamepad2.right_bumper;

        double drive; // drive: left joystick y-axis
        double turn;  // turn: right joystick x-axis
        double strafe;  // strafe: left joystick x-axis
        double drive_speed = 0.8;
        double turn_speed = 0.5;
        double strafe_speed = 0.5;
        double FLspeed, FRspeed, BLspeed, BRspeed;

        //Robot Hardware Mapping:
        FrontLeft = hardwareMap.get(DcMotor.class, flName);
        FrontRight = hardwareMap.get(DcMotor.class, frName);
        BackLeft = hardwareMap.get(DcMotor.class, blName);
        BackRight = hardwareMap.get(DcMotor.class, brName);

        claw = hardwareMap.get(Servo.class, "claw2");
        y = gamepad2.y;
        b= gamepad2.b;
        x = gamepad2.x;
        telemetry.update();

        //Robot Hardware Mapping:
        LeftLinearSlide = hardwareMap.dcMotor.get(LeftLinearSlideName);
        RightLinearSlide = hardwareMap.dcMotor.get(RightLinearSlideName);

        // Robot Hardware Configuration:
        //For Drive Train
        // Reverse Left side motors.
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);


        //STOP_AND_RESET_ENCODER
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //BRAKE-- need to check...
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SET Run without encoder
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            //Drive Train code Start===============================================================
            drive = (gamepad1.left_stick_y * -1) * drive_speed;
            turn = (gamepad1.right_stick_x) * turn_speed;
            strafe = (gamepad1.left_stick_x) * strafe_speed;

            FLspeed = drive + turn + strafe;
            FRspeed = drive - turn - strafe;
            BLspeed = drive + turn - strafe;
            BRspeed = drive - turn + strafe;

            // Scaling Drive Powers Proportionally
            double maxF = Math.max((abs(FLspeed)),(abs(FRspeed)));
            double maxB = Math.max((abs(BLspeed)),(abs(BRspeed)));
            double maxFB_speed = Math.max(abs(maxF), abs(maxB));

            if(maxFB_speed > 1){
                FLspeed = FLspeed / maxFB_speed;
                FRspeed = FRspeed / maxFB_speed;
                BLspeed = BLspeed / maxFB_speed;
                BRspeed = BRspeed / maxFB_speed;
            }

            FrontLeft.setPower(FLspeed);
            FrontRight.setPower(FRspeed);
            BackLeft.setPower(BLspeed);
            BackRight.setPower(BRspeed);


            // Drive Train code end




        if (gamepad2.right_bumper&&!ClawClosed) {
            claw.setPosition(0.3);//close
            sleep(250);
            ClawClosed = true;
        }
        if (gamepad2.right_bumper&&ClawClosed) {
            claw.setPosition(0);//open
            sleep(250);
            ClawClosed = false;
        }


        if (x && !ChannelSLidesExtended) {
            ChannelSlides.setPower(1);
            sleep(1500);
                ChannelSLidesExtended = true;
                ChannelSlides.setPower(0);

        }


        if(x && ChannelSLidesExtended){
            ChannelSlides.setPower(-1);
            sleep(1500);
                ChannelSLidesExtended = false;
                ChannelSlides.setPower(0);
        }


        if (gamepad2.y) {
            // if (ClawPivotPos>=0&&ClawPivotPos<=4) {
            ClawPivotPos = ClawPivotPos + 1;
            ClawPivotPos2 = ClawPivotPos / 8;
            clawPivot.setPosition(ClawPivotPos2);
            sleep(250);
//             //  }else if(ClawPivotPos>4){
//                   ClawPivotPos = 4;
//                   ClawPivotPos2 = ClawPivotPos/8;
//             //  }else if(ClawPivotPos<0){
//                   ClawPivotPos = 0;
//                   ClawPivotPos2 = ClawPivotPos/8;
        }
        if (gamepad2.b) {
            //   if (ClawPivotPos >= 0 && ClawPivotPos <= 4) {
            ClawPivotPos = ClawPivotPos - 1;
            ClawPivotPos2 = ClawPivotPos / 8;
            clawPivot.setPosition(ClawPivotPos2);
            sleep(250);
//               // } else if (ClawPivotPos > 4) {
//                    ClawPivotPos = 8;
//                    ClawPivotPos2 = ClawPivotPos / 8;
//               // } else if (ClawPivotPos < 0) {
//                    ClawPivotPos = 0;
//                    ClawPivotPos2 = ClawPivotPos / 8;
            //}
            // }
        }

            // For Linear Slide Code start==========================================================
            if ((gamepad2.dpad_down) && (!LS_motorRunning)) {
                LS_value = 0;
                LS_TargetPosition = (int) (LS_value * LS_TICKS_PER_MM);
                LeftLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
                LS_motorRunning = true;
            }

            if ((gamepad2.dpad_right) && (!LS_motorRunning)) {
                LS_value = 75;
                LS_TargetPosition = ((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
                LS_motorRunning = true;
            }
            if ((gamepad2.dpad_up) && (!LS_motorRunning)) {
                LS_value = 700;
                LS_TargetPosition = ((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
                LS_motorRunning = true;
            }
            if ((gamepad2.dpad_left) && (!LS_motorRunning)) {
                LS_value = 670;
                LS_TargetPosition = ((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
                LS_motorRunning = true;
            }


            if (Math.abs(LeftLinearSlide.getCurrentPosition() - LS_TargetPosition) < LS_TOLERANCE && LS_TargetPosition != LS_RESET_POSITION) {
                LeftLinearSlide.setPower(LS_half_speed);
                RightLinearSlide.setPower(LS_half_speed);
                LS_motorRunning = false;
            }
            if (Math.abs(RightLinearSlide.getCurrentPosition() - LS_TargetPosition) < LS_TOLERANCE && LS_TargetPosition != LS_RESET_POSITION) {
                LeftLinearSlide.setPower(LS_half_speed);
                RightLinearSlide.setPower(LS_half_speed);
                LS_motorRunning = false;
            }

            if ((LeftLinearSlide.getCurrentPosition() >= LS_RESET_POSITION && LeftLinearSlide.getCurrentPosition() <= 10) && LS_TargetPosition == LS_RESET_POSITION) {
                LeftLinearSlide.setPower(0);
                RightLinearSlide.setPower(0);
                LS_motorRunning = false;
            }
            if ((RightLinearSlide.getCurrentPosition() >= LS_RESET_POSITION && RightLinearSlide.getCurrentPosition() <= 10) && LS_TargetPosition == LS_RESET_POSITION) {
                LeftLinearSlide.setPower(0);
                RightLinearSlide.setPower(0);
                LS_motorRunning = false;
            }
            // For Linear Slide Code end


        }
    }
}
