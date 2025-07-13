package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ETAT_DT_LS_Arm2_v1", group = "ETAT")
public class ETAT_DT_LS_Arm2_v1 extends LinearOpMode {
    //Hardware Map Names
    String flName = "fl", frName = "fr", blName = "bl", brName = "br";
    String LeftLinearSlideName = "lsl", RightLinearSlideName = "lsr";
    String ClawServoName = "claw";
    String ArmName = "arm";

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
//    final double LS_SCORING_IN_HIGH_BASKET = 483 * LS_TICKS_PER_MM;  // 480 -- 483 --485
    final double LS_SCORING_IN_HIGH_BASKET = 483 * LS_TICKS_PER_MM;  // 480 -- 483 --485
    final double LS_MAX_MOTOR_CURRENT = 2.0;

    double LS_full_speed = 0.8;   //0.8 or 1.0
    double LS_half_speed = LS_full_speed/3;   // div 2
    int LS_TargetPosition = (int)LS_COLLAPSED;
    boolean LS_motorRunning = false;
    double LS_RESET_POSITION = 0;
    double LS_TOLERANCE = 2.0;

//    //FOR ARM1 Motor: we are using 30RPM motor
//    final double ARM_TICKS_PER_DEGREE =
//            28 // number of encoder ticks per rotation of the bare motor
//                    * 188.0 / 1.0 // This is the exact gear ratio of the 30RPM 188:1 Yellow Jacket gearbox
//                    * 24.0 / 24.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
//                    * 1/360.0; // we want ticks per degree, not per rotation

    //FOR ARM2 Motor: We are using 117RPM motor
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    public DcMotor armMotor = null;

    final double ARM_COLLAPSED_INTO_ROBOT  = 0 * ARM_TICKS_PER_DEGREE;
//    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
//    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
//    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    final double ARM_ATTACH_HANGING_HOOK   = 175 * ARM_TICKS_PER_DEGREE;  //166
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;  //235
    final double ARM_PICKUP               = 252 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLOSER_TO_REST       = 60  * ARM_TICKS_PER_DEGREE;
    final double ARM_CLIP                 = 210 * ARM_TICKS_PER_DEGREE;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 25 * ARM_TICKS_PER_DEGREE; //15

    double armTargetPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    double arm_full_speed = 0.8;
    double arm_half_speed = arm_full_speed/2;
    boolean arm_motorRunning = false;
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
        LeftLinearSlide = hardwareMap.dcMotor.get(LeftLinearSlideName);
        RightLinearSlide = hardwareMap.dcMotor.get(RightLinearSlideName);
        Claw_Servo = hardwareMap.get(Servo.class,ClawServoName);
        armMotor = hardwareMap.dcMotor.get(ArmName);

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
       // RightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // For Arm
//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // For Arm
        armMotor.setTargetPosition((int)(ARM_COLLAPSED_INTO_ROBOT));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setPower(0);

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

            // For Linear Slide Code start==========================================================
            if ((gamepad2.dpad_up) && (!LS_motorRunning)){
//                clawOperation = CLAW_CLOSE;
//                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//                Claw_Servo.setPosition(clawOperation);
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
//                clawOperation = CLAW_OPEN;
//                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//                Claw_Servo.setPosition(clawOperation);
//                sleep(500);
                LS_TargetPosition = (int)(LS_COLLAPSED);
                LeftLinearSlide.setTargetPosition((int)(LS_COLLAPSED));
                RightLinearSlide.setTargetPosition((int)(LS_COLLAPSED));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);

                while(LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()){idle();}

                armTargetPosition = ((int) (175 * ARM_TICKS_PER_DEGREE));
                armMotor.setTargetPosition((int) (armTargetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);
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
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                Claw_Servo.setPosition(clawOperation);
            }
            else if (leftBumper) {
//                clawOperation = CLAW_OPEN_S;
//                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//                Claw_Servo.setPosition(clawOperation);
                //new code need to test....
                if (LeftLinearSlide.getCurrentPosition() > (200 * LS_TICKS_PER_MM) || RightLinearSlide.getCurrentPosition() > (200 * LS_TICKS_PER_MM)){
                    clawOperation = CLAW_OPEN_S;
                    clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                    Claw_Servo.setPosition(clawOperation);
                    sleep(1000);
                    armTargetPosition = ((int) (160 * ARM_TICKS_PER_DEGREE));
                    armMotor.setTargetPosition((int) (armTargetPosition));
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(arm_full_speed);
                }else {
                    clawOperation = CLAW_OPEN_S;
                    clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                    Claw_Servo.setPosition(clawOperation);
                }
            }
            // For Claw code end

//            //For Arm code start==================================================================
            if ((gamepad2.back) && (!arm_motorRunning)){
                clawOperation = CLAW_OPEN;
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                Claw_Servo.setPosition(clawOperation);
                sleep(500);
                armTargetPosition = ((int) (ARM_COLLAPSED_INTO_ROBOT));
                armMotor.setTargetPosition((int) (armTargetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);

//                while (armMotor.isBusy()){
//                    idle();
//                }
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setPower(0);
                arm_motorRunning = true;
            }

            if ((gamepad2.y) && (!arm_motorRunning)){
                armTargetPosition = ((int) (ARM_ATTACH_HANGING_HOOK));
                armMotor.setTargetPosition((int) (armTargetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);
                arm_motorRunning = true;
            }

            if ((gamepad2.b) && (!arm_motorRunning)){
                armTargetPosition = ((int) (ARM_CLEAR_BARRIER));
                armMotor.setTargetPosition((int) (armTargetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);
                arm_motorRunning = true;
            }

            if ((gamepad2.a) && (!arm_motorRunning)){
                armTargetPosition = ((int) (ARM_PICKUP));
                armMotor.setTargetPosition((int) (armTargetPosition));
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(arm_full_speed);
                arm_motorRunning = true;
            }

//            if ((gamepad2.x) && (!arm_motorRunning)){
//                armTargetPosition = ((int) (ARM_CLIP));
//                armMotor.setTargetPosition((int) (armTargetPosition));
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor.setPower(arm_full_speed);
//                arm_motorRunning = true;
//            }

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));
            armMotor.setTargetPosition((int) (armTargetPosition + armPositionFudgeFactor));
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(arm_full_speed);

            if (Math.abs(armMotor.getCurrentPosition() - armTargetPosition) < ARM_TOLERANCE && armTargetPosition!= ARM_RESET_POSITION) {
                armMotor.setPower(arm_half_speed);
                arm_motorRunning=false;
            }

            if((armMotor.getCurrentPosition()>=ARM_RESET_POSITION && armMotor.getCurrentPosition()<=10) && armTargetPosition==ARM_RESET_POSITION ) {
                armMotor.setPower(0);
                arm_motorRunning=false;
            }

//            telemetry.addData("Arm Target Position = ",armMotor.getTargetPosition());
//            telemetry.addData("Arm current position =", armMotor.getCurrentPosition());
//            telemetry.addData("Arm Motor Current =",((DcMotorEx) armMotor).getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Arm Motor Power = ", armMotor.getPower());
//            telemetry.update();
            //For Arm code End==================================================================
      }
    }
}
