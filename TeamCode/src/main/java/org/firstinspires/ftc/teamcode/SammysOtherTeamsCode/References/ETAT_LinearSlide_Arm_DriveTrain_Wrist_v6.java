package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "ETAT_LinearSlide_Arm_DriveTrain_Wrist_v6", group = "ETAT")
public class ETAT_LinearSlide_Arm_DriveTrain_Wrist_v6 extends LinearOpMode {
    LED LEDname;
    //Hardware Map Names
    String flName = "fl", frName = "fr", blName = "bl", brName = "br";
    String LeftLinearSlideName = "lsl", RightLinearSlideName = "lsr";
    String ClawServoName = "claw";
    String ArmName = "arm";
    String WristServoName = "wrist";
    String LEDName = "ledgj";

    // Define or Declare Hardware here...
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor BackLeft = null;
    public DcMotor BackRight = null;

    // For linear slides
    public DcMotor LeftLinearSlide = null;
    public DcMotor RightLinearSlide = null;

    final double LS_TICKS_PER_MM = 537.7 / 120.0;
    final double LS_COLLAPSED = 0 * LS_TICKS_PER_MM;
    final double LS_SCORING_IN_LOW_BASKET = 0 * LS_TICKS_PER_MM;
    final double LS_SCORING_IN_HIGH_BASKET = 483 * LS_TICKS_PER_MM;  // 480
    final double LS_MAX_MOTOR_CURRENT = 2.0;

    double LS_full_speed = 1.0;   //0.8
    double LS_half_speed = LS_full_speed/3;   // div 2
//    final double LS_GOING_UP_SPEED = 0.7;
//    final double LS_GOING_DOWN_SPEED = 0.7;
//    double LS_STOP = 0;

//    boolean LS_top_flag = false;
//    boolean LS_up_flag = false;  // 0= false, 1=true
//    boolean LS_down_flag = false;  // 0= false, 1=true

//    final int LS_GROUND_LEVEL = 0;

    int LS_TargetPosition = (int)LS_COLLAPSED;
    //    double LS_speed;
    boolean LS_motorRunning = false;
    double LS_RESET_POSITION = 0;
    double LS_TOLERANCE = 2.0;


    // For Arm motor
//    final double ARM_TICKS_PER_DEGREE =
//            28 // number of encoder ticks per rotation of the bare motor
//                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
////                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
//                    * 24.0 / 24.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
//                    * 1/360.0; // we want ticks per degree, not per rotation
    final double ARM_TICKS_PER_DEGREE =
            5281.1 // number of encoder Encoder Resolution for 5203-2402-0188 == 188:1 Ratio, 24mm Length 8mm REX™ Shaft, 30 RPM, 3.3 - 5V Encoder
                    * 24.0 / 24.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    public DcMotor armMotor = null;

    final double ARM_COLLAPSED_INTO_ROBOT  = 0 * ARM_TICKS_PER_DEGREE;
    //    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
//    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    //    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    final double ARM_ATTACH_HANGING_HOOK   = 166 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 235 * ARM_TICKS_PER_DEGREE;
    final double ARM_PICKUP               = 255 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLOSER_TO_REST       = 60  * ARM_TICKS_PER_DEGREE;


    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; //15
//    final double FUDGE_FACTOR = 10;

    double armTragetPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    //    double armSpeed;
    double armPositionFudgeFactor;

    double arm_full_speed = 0.8;
    double arm_half_speed = arm_full_speed/2;
    boolean arm_flag = false;
    boolean button_flag = false;
    boolean Button_BACK_flag = false;
    boolean Button_Y_flag = false;
    boolean artestRunning = false;
    double ARM_RESET_POSITION = 0;
    double ARM_TOLERANCE = 2.0;

    // For Claw
    public Servo Claw_Servo = null;
    public final static double CLAW_HOME = 0.0;
    public final static double CLAW_MIN_RANGE = 0.0;
    public final static double CLAW_MAX_RANGE = 0.5;
    public final static double CLAW_CLOSE = 0.0;
    public final static double CLAW_OPEN = 0.25;
    public final static double CLAW_OPEN_S = 0.17;
    double clawOperation = CLAW_HOME;

    // For Wrist
    public Servo Wrist_Servo = null;
    public final static double WRIST_AT_BACK_FACING_DOWN = 0.015;
    public final static double WRIST_AT_BACK_FACING_UP = 0.665;
    public final static double WRIST_MIN_RANGE = 0.0;
    public final static double WRIST_MAX_RANGE = 0.70;
    public final static double WRIST_HOME = WRIST_AT_BACK_FACING_UP;
    double wristOperation = WRIST_HOME;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Telemetry.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
//        LeftLinearSlide = hardwareMap.get(DcMotor.class, LeftLinearSlideName);
//        RightLinearSlide = hardwareMap.get(DcMotor.class, RightLinearSlideName);
        LeftLinearSlide = hardwareMap.dcMotor.get(LeftLinearSlideName);
        RightLinearSlide = hardwareMap.dcMotor.get(RightLinearSlideName);
        Claw_Servo = hardwareMap.get(Servo.class,ClawServoName);
//        armMotor = hardwareMap.get(DcMotor.class, ArmName);
        armMotor = hardwareMap.dcMotor.get(ArmName);
        Wrist_Servo = hardwareMap.get(Servo.class,WristServoName);
        LEDname = hardwareMap.get(LED.class, LEDName);

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

        // For Arm
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initial conditions:
        //For Drive Train
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        //For Linear SLide
        LS_motorRunning = false;
        LS_TargetPosition = (int) (LS_COLLAPSED);

        LeftLinearSlide.setTargetPosition((int) (LS_COLLAPSED));
        RightLinearSlide.setTargetPosition((int) (LS_COLLAPSED));
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //For Claw
        Claw_Servo.setPosition(CLAW_OPEN);

        //For Wrist
        Wrist_Servo.setPosition(WRIST_HOME);

        //For LED
        LEDname.enableLight(false);

        // For Arm
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition((int)(ARM_COLLAPSED_INTO_ROBOT));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setPower(0);
        arm_flag = false;
        button_flag = false;
        Button_BACK_flag = false;
        Button_Y_flag = false;

        telemetry.addLine("Initialization & Configuration : > Wait for Start ");
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

            // For Linear Slide Code start==========================================================
            if ((gamepad2.dpad_up) && (!LS_motorRunning)){
                clawOperation = CLAW_CLOSE;
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                Claw_Servo.setPosition(clawOperation);
                LS_TargetPosition = ((int) (LS_SCORING_IN_HIGH_BASKET));
                LeftLinearSlide.setTargetPosition((int) (LS_SCORING_IN_HIGH_BASKET));
                RightLinearSlide.setTargetPosition((int) (LS_SCORING_IN_HIGH_BASKET));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
                LS_motorRunning = true;
            }

            if ((gamepad2.dpad_down) && (!LS_motorRunning)){
                LEDname.enableLight(false);
                clawOperation = CLAW_OPEN;
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                Claw_Servo.setPosition(clawOperation);
                sleep(500);
                LS_TargetPosition = (int)(LS_COLLAPSED);
                LeftLinearSlide.setTargetPosition((int)(LS_COLLAPSED));
                RightLinearSlide.setTargetPosition((int)(LS_COLLAPSED));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
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

//            telemetry.addData("LSL Target Position = ",LeftLinearSlide.getTargetPosition());
//            telemetry.addData("LSL current position =", LeftLinearSlide.getCurrentPosition());
//            telemetry.addData("LSL Motor Current =",((DcMotorEx) LeftLinearSlide).getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("LSL Motor Power = ", LeftLinearSlide.getPower());
//            telemetry.addLine("==============================");
//            telemetry.addData("LSR Target Position = ",RightLinearSlide.getTargetPosition());
//            telemetry.addData("LSR current position =", RightLinearSlide.getCurrentPosition());
//            telemetry.addData("LSR Motor Current =",((DcMotorEx) RightLinearSlide).getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("LSR Motor Power = ", RightLinearSlide.getPower());
//            telemetry.update();

            // For Linear Slide Code end

            //For Claw code start==================================================================
            boolean rightBumper = gamepad2.right_bumper;
            boolean leftBumper = gamepad2.left_bumper;
            if (rightBumper) {
                clawOperation = CLAW_CLOSE;
            }
            else if (leftBumper) {
//                clawOperation = CLAW_OPEN;
                clawOperation = CLAW_OPEN_S;
            }
            clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
            Claw_Servo.setPosition(clawOperation);
            // For Claw code end

            //For Wrist code start==================================================================
//            if (gamepad2.right_trigger > 0.2) {
//               wristOperation = 0.015; // WRIST_AT_BACK_FACING_DOWN = 0.015
//            }
//            else if (gamepad2.left_trigger > 0.2) {
//                wristOperation = 0.665; //WRIST_AT_FRONT_FACING_UP = 0.665
//            }
            wristOperation = Range.clip(wristOperation, WRIST_MIN_RANGE, WRIST_MAX_RANGE);
            Wrist_Servo.setPosition(wristOperation);
            // For Claw code end

//            //For Arm code start==================================================================
            if ((gamepad2.back) && (!artestRunning)){
                LEDname.enableLight(false);
                clawOperation = CLAW_OPEN;
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                Claw_Servo.setPosition(clawOperation);
                sleep(500);
                armTragetPosition = ((int) (ARM_COLLAPSED_INTO_ROBOT));
                armMotor.setTargetPosition((int) (armTragetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);

                artestRunning = true;
            }

            if ((gamepad2.y) && (!artestRunning)){
                LEDname.enableLight(false);
//                clawOperation = CLAW_CLOSE;
//                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//                Claw_Servo.setPosition(clawOperation);
////                sleep(500);
                armTragetPosition = ((int) (ARM_ATTACH_HANGING_HOOK));
                armMotor.setTargetPosition((int) (armTragetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);
                artestRunning = true;
            }

            if ((gamepad2.b) && (!artestRunning)){
//                clawOperation = CLAW_OPEN;
//                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//                Claw_Servo.setPosition(clawOperation);

                LEDname.enableLight(true);

                armTragetPosition = ((int) (ARM_CLEAR_BARRIER));
                armMotor.setTargetPosition((int) (armTragetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);
                artestRunning = true;
            }

            if ((gamepad2.a) && (!artestRunning)){
                LEDname.enableLight(false);
//                clawOperation = CLAW_OPEN;
//                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//                Claw_Servo.setPosition(clawOperation);

                armTragetPosition = ((int) (ARM_PICKUP));
                armMotor.setTargetPosition((int) (armTragetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);
//                sleep(500);
//                clawOperation = CLAW_CLOSE;
//                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//                Claw_Servo.setPosition(clawOperation);
//                sleep(500);
//                armTragetPosition = ((int) (ARM_CLEAR_BARRIER));
//                armMotor.setTargetPosition((int) (armTragetPosition));
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor.setPower(arm_full_speed);

                artestRunning = true;
            }
            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));
            armMotor.setTargetPosition((int) (armTragetPosition + armPositionFudgeFactor));
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(arm_full_speed);

            if (Math.abs(armMotor.getCurrentPosition() - armTragetPosition) < ARM_TOLERANCE && armTragetPosition!= ARM_RESET_POSITION) {
                armMotor.setPower(arm_half_speed);
                artestRunning=false;
            }

            if((armMotor.getCurrentPosition()>=ARM_RESET_POSITION && armMotor.getCurrentPosition()<=10) && armTragetPosition==ARM_RESET_POSITION ) {
                armMotor.setPower(0);
                artestRunning=false;
            }

//            telemetry.addData("Arm Target Position = ",armMotor.getTargetPosition());
//            telemetry.addData("Arm current position =", armMotor.getCurrentPosition());
//            telemetry.addData("Arm Motor Current =",((DcMotorEx) armMotor).getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Arm Motor Power = ", armMotor.getPower());
//            telemetry.update();
            //For Arm code End==================================================================
        }
        LEDname.enableLight(false);
    }

}
