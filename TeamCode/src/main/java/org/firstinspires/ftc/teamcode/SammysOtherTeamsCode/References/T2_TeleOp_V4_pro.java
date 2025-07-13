// CONFIG : FILE name "T2-CONFIG"
//-------------------------------------------------
// CONTROL HUB :
// 1) MOTOR:
// - M0 = lsl  (Use encoder cable)
// - M1 = lsr  (Use encoder cable)
// - M2 =
// - M3 =
// 2) SERVO:
// - port0 = specimenclaw
// - port1 = szoneslide
// - port2 = wrist
// - port3 = sampleclaw
//-------------------------------------------------
// EXPANSION HUB :
// 1) MOTOR:
// - M0 = fl (Don't use encoder cable)
// - M1 = fr (Don't use encoder cable)
// - M2 = bl (Don't use encoder cable)
// - M3 = br (Don't use encoder cable)
// 2) SERVO:
// - port0 =
// - port1 =
// - port2 =
// - port3 =
//##############################################################
package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "T2_TeleOp_V4_pro", group = "ETAT")
public class T2_TeleOp_V4_pro extends LinearOpMode {
    //Hardware Map Names
    String flName = "fl", frName = "fr", blName = "bl", brName = "br";
    String LeftLinearSlideName = "lsl", RightLinearSlideName = "lsr";
    String SpecimenClawName = "specimenclaw";
    String SzonelideName = "szoneslide";
    String WristName = "wrist";
    String SampleClawName = "sampleclaw";

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
    double LS_half_speed = LS_full_speed / 2;   // div 2

    double LS_OneThird_speed = LS_full_speed / 3;   // div 3
    boolean LS_motorRunning = false;
    double LS_RESET_POSITION = 0;
    double LS_TOLERANCE = 2.0;

    // For Specimen_Claw
    public Servo Specimen_Claw = null;
    public final static double SPECIMEN_CLAW_HOME = 0.0;
    public final static double SPECIMEN_CLAW_CLOSE = 0.0;
    public final static double SPECIMEN_CLAW_OPEN = 0.25;

    public final static double SPECIMEN_CLAW_OPEN_first = 0.35;

    public final static double SPECIMEN_CLAW_OPEN_S = 0.3;
    private boolean Specimen_Claw_Open = true;
    private boolean Specimen_Claw_lastBump = false;


    // For Szone Slide
//    public CRServo SzoneSlide = null;
    public Servo SzoneSlide = null;
    boolean SzoneSlideExtended = false;
    private boolean SzoneSlide_lastBump = false;


    //For Wrist
    public Servo Wrist = null;

    double WP1 = 0.0;
    double WP2 = 0.1625;
    double WP3 = 0.325;
    double WP4 = 0.4875;
    double WP5 = 0.65;

    //For SAMPLE CLAW
    public Servo Sample_Claw = null;
    public final static double SAMPLE_CLAW_HOME = 0.0;
    public final static double SAMPLE_CLAW_CLOSE = 0.0;
    public final static double SAMPLE_CLAW_CLOSE_S = 0.15;
    public final static double SAMPLE_CLAW_OPEN = 0.25;
    public final static double SAMPLE_CLAW_OPEN_S = 0.4;
    //    double SampleClawOperation = SAMPLE_CLAW_HOME;
    private boolean Sample_Claw_Open = true;
    private boolean Sample_Claw_lastBump = false;




    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Telemetry.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double drive; // drive: left joystick y-axis
        double turn;  // turn: right joystick x-axis
        double strafe;  // strafe: left joystick x-axis
        double drive_speed = 1.0; // 0.8
        double turn_speed = 0.75;  // 0.5
        double strafe_speed = 0.75; //0.5
        double FLspeed, FRspeed, BLspeed, BRspeed;

        //Robot Hardware Mapping:
        FrontLeft = hardwareMap.get(DcMotor.class, flName);
        FrontRight = hardwareMap.get(DcMotor.class, frName);
        BackLeft = hardwareMap.get(DcMotor.class, blName);
        BackRight = hardwareMap.get(DcMotor.class, brName);
        LeftLinearSlide = hardwareMap.dcMotor.get(LeftLinearSlideName);
        RightLinearSlide = hardwareMap.dcMotor.get(RightLinearSlideName);
        Specimen_Claw = hardwareMap.get(Servo.class, SpecimenClawName);
//        SzoneSlide = hardwareMap.get(CRServo.class, SzonelideName);
        SzoneSlide = hardwareMap.get(Servo.class, SzonelideName);
        Wrist = hardwareMap.get(Servo.class, WristName);
        Sample_Claw = hardwareMap.get(Servo.class, SampleClawName);

        // Robot Hardware Configuration:
        //For Drive Train
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //STOP_AND_RESET_ENCODER
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //SET Run without encoder
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //For Linear SLide
        RightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initial conditions:
        //For Drive Train
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        //For Linear SLide
        LS_motorRunning = false;
        int LS0_value = 0;
        int LS_TargetPosition = (int) (LS0_value * LS_TICKS_PER_MM);
        LeftLinearSlide.setTargetPosition((int) (LS0_value * LS_TICKS_PER_MM));
        RightLinearSlide.setTargetPosition((int) (LS0_value * LS_TICKS_PER_MM));
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //For SPECIMEN CLAW
        Specimen_Claw.setPosition(SPECIMEN_CLAW_OPEN_first);

        //For Szone Slide
//        SzoneSlide.setPower(0);
        SzoneSlide.setPosition(0);


        //For Wrist
        double WristPOS = WP3;
        boolean WristClockwise = true;
        Wrist.setPosition(WristPOS);

        //For SAMPLE CLAW
        Sample_Claw.setPosition(SAMPLE_CLAW_HOME);

        telemetry.addLine("TELEOP: Press START > button");
        telemetry.addLine("GP1-Left-Stick Y = Drive Forward/Backward");
        telemetry.addLine("GP1-Right-Stick X = Turn Right/Left");
        telemetry.addLine("GP1-Left-Stick X = Strafe Right/Left");
        telemetry.addLine("GP1-Dpad-Down = SZONE Entry Level");
        telemetry.addLine("GP1-Dpad-Up = Robot Hang Level");
        telemetry.addLine("GP1-Dpad-Right = Hang Robot");
        telemetry.addLine("==========================================");
        telemetry.addLine("GP2-X = SZONE Slide Forward/Backward");
        telemetry.addLine("GP2-Y = Sample Claw rotate 45 Deg");
        telemetry.addLine("GP2-A = SZONE slide Down");
        telemetry.addLine("GP2-B = SZONE slide Up");
        telemetry.addLine("GP2-Bumper-Right = Specimen Claw OPEN/CLOSE");
        telemetry.addLine("GP2-Bumper-Left = Sample Claw OPEN/CLOSE");
        telemetry.addLine("GP2-Dpad-Down = PickUP Specimen Level");
        telemetry.addLine("GP2-Dpad-Up = High Chamber Level");
        telemetry.addLine("GP2-Dpad-Right = Hang Specimen");
        telemetry.addLine("GP2-Dpad-Left = Low Basket Drop Sample");
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
            double maxF = Math.max((abs(FLspeed)), (abs(FRspeed)));
            double maxB = Math.max((abs(BLspeed)), (abs(BRspeed)));
            double maxFB_speed = Math.max(abs(maxF), abs(maxB));

            if (maxFB_speed > 1) {
                FLspeed = FLspeed / maxFB_speed;
                FRspeed = FRspeed / maxFB_speed;
                BLspeed = BLspeed / maxFB_speed;
                BRspeed = BRspeed / maxFB_speed;
            }

            FrontLeft.setPower(FLspeed);
            FrontRight.setPower(FRspeed);
            BackLeft.setPower(BLspeed);
            BackRight.setPower(BRspeed);
            // Drive Train code end=================================================================

            // For Linear Slide Code start==========================================================
            if ((gamepad2.dpad_down) && (!LS_motorRunning)) {
                Sample_Claw.setPosition(SAMPLE_CLAW_OPEN_S);
                LS0_value = 0;
                LS_TargetPosition = (int) (LS0_value * LS_TICKS_PER_MM);
                LeftLinearSlide.setTargetPosition((int) (LS0_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS0_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
//                while (LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()) {
//                    idle();
//                }
                LS_motorRunning = true;
                Sample_Claw.setPosition(SAMPLE_CLAW_CLOSE_S);
            }

            if ((gamepad2.a) && (!LS_motorRunning)) {
//                LS0_value = 0;
                LS_TargetPosition = (int) (20 * LS_TICKS_PER_MM);
                LeftLinearSlide.setTargetPosition((int) (20 * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (20 * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
//                while (LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()) {
//                    idle();
//                }
                LS_motorRunning = true;
                Sample_Claw.setPosition(SAMPLE_CLAW_CLOSE_S);
            }

            if ((gamepad2.dpad_up) && (!LS_motorRunning)) {
                int LS3_value = 415;  //470
                LS_TargetPosition = ((int) (LS3_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS3_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS3_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
//                while (LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()) {
//                    idle();
//                }
                LS_motorRunning = true;
            }

            if ((gamepad2.dpad_right) && (!LS_motorRunning)) {
                int LS2_value = 290;  //350
                LS_TargetPosition = ((int) (LS2_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS2_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS2_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
//                while (LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()) {
//                    idle();
//                }
                LS_motorRunning = true;
                Sample_Claw.setPosition(SAMPLE_CLAW_CLOSE_S);

            }

//            if ((gamepad2.dpad_left) && (!LS_motorRunning)) {
//                int LS5_value = 450;
//                LS_TargetPosition = ((int) (LS5_value * LS_TICKS_PER_MM));
//                LeftLinearSlide.setTargetPosition((int) (LS5_value * LS_TICKS_PER_MM));
//                RightLinearSlide.setTargetPosition((int) (LS5_value * LS_TICKS_PER_MM));
//                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                LeftLinearSlide.setPower(LS_full_speed);
//                RightLinearSlide.setPower(LS_full_speed);
////                while (LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()) {
////                    idle();
////                }
//                LS_motorRunning = true;
//            }

            if ((gamepad2.b) && (!LS_motorRunning)) {
                int LS1_value = 95;
                LS_TargetPosition = ((int) (LS1_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS1_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS1_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
//                while (LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()) {
//                    idle();
//                }
                LS_motorRunning = true;
//                Sample_Claw.setPosition(SAMPLE_CLAW_OPEN_S);
            }

            if ((gamepad1.dpad_down) && (!LS_motorRunning)) {
                Sample_Claw.setPosition(SAMPLE_CLAW_OPEN_S);
                int LS1_value = 85;
                LS_TargetPosition = ((int) (LS1_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setTargetPosition((int) (LS1_value * LS_TICKS_PER_MM));
                RightLinearSlide.setTargetPosition((int) (LS1_value * LS_TICKS_PER_MM));
                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftLinearSlide.setPower(LS_full_speed);
                RightLinearSlide.setPower(LS_full_speed);
//                while (LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()) {
//                    idle();
//                }
                LS_motorRunning = true;
            }

//            if ((gamepad1.dpad_up) && (!LS_motorRunning)) {
//                Sample_Claw.setPosition(SAMPLE_CLAW_OPEN_S);
//                int LS4_value = 550;
//                LS_TargetPosition = ((int) (LS4_value * LS_TICKS_PER_MM));
//                LeftLinearSlide.setTargetPosition((int) (LS4_value * LS_TICKS_PER_MM));
//                RightLinearSlide.setTargetPosition((int) (LS4_value * LS_TICKS_PER_MM));
//                LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                LeftLinearSlide.setPower(LS_full_speed);
//                RightLinearSlide.setPower(LS_full_speed);
////                while (LeftLinearSlide.isBusy() || RightLinearSlide.isBusy()) {
////                    idle();
////                }
//                LS_motorRunning = true;
//            }

            if (Math.abs(LeftLinearSlide.getCurrentPosition() - LS_TargetPosition) < LS_TOLERANCE && LS_TargetPosition != LS_RESET_POSITION) {
                LeftLinearSlide.setPower(LS_OneThird_speed);
                RightLinearSlide.setPower(LS_OneThird_speed);
                LS_motorRunning = false;
            }
            if (Math.abs(RightLinearSlide.getCurrentPosition() - LS_TargetPosition) < LS_TOLERANCE && LS_TargetPosition != LS_RESET_POSITION) {
                LeftLinearSlide.setPower(LS_OneThird_speed);
                RightLinearSlide.setPower(LS_OneThird_speed);
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
            // For Linear Slide Code end ===========================================================

            //For Specimen_Claw code start===================================================================
            if (gamepad2.right_bumper && !Specimen_Claw_lastBump) {
                Specimen_Claw_Open = !Specimen_Claw_Open;
                if (Specimen_Claw_Open) {
                    Specimen_Claw.setPosition(SPECIMEN_CLAW_OPEN_S);
                } else {
                    Specimen_Claw.setPosition(SPECIMEN_CLAW_CLOSE);
                }
            }
            Specimen_Claw_lastBump = gamepad2.right_bumper;
            // For Specimen_Claw code end====================================================================

            //For Sample Claw code start======================================================================
            if (gamepad2.left_bumper && !Sample_Claw_lastBump) {
                Sample_Claw_Open = !Sample_Claw_Open;
                if (Sample_Claw_Open) {
                    Sample_Claw.setPosition(SAMPLE_CLAW_OPEN_S);
                } else {
                    Sample_Claw.setPosition(SAMPLE_CLAW_CLOSE_S);
                }
            }
            Sample_Claw_lastBump = gamepad2.left_bumper;
            //For Sample Claw code End======================================================================

            //For Szone Slides code start======================================================================
            if (gamepad2.x && !SzoneSlide_lastBump) {
                SzoneSlideExtended = !SzoneSlideExtended;
                if (SzoneSlideExtended) {
                    LS_TargetPosition = ((int) (70 * LS_TICKS_PER_MM));
                    LeftLinearSlide.setTargetPosition((int) (70 * LS_TICKS_PER_MM));
                    RightLinearSlide.setTargetPosition((int) (70 * LS_TICKS_PER_MM));
                    LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftLinearSlide.setPower(LS_full_speed);
                    RightLinearSlide.setPower(LS_full_speed);

                    SzoneSlide.setPosition(0.48);
                } else {
                    LS_TargetPosition = ((int) (70 * LS_TICKS_PER_MM));
                    LeftLinearSlide.setTargetPosition((int) (70 * LS_TICKS_PER_MM));
                    RightLinearSlide.setTargetPosition((int) (70 * LS_TICKS_PER_MM));
                    LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftLinearSlide.setPower(LS_full_speed);
                    RightLinearSlide.setPower(LS_full_speed);

                    SzoneSlide.setPosition(-0.48);
                    sleep(800);

                    LS_TargetPosition = ((int) (0 * LS_TICKS_PER_MM));
                    LeftLinearSlide.setTargetPosition((int) (0 * LS_TICKS_PER_MM));
                    RightLinearSlide.setTargetPosition((int) (0 * LS_TICKS_PER_MM));
                    LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftLinearSlide.setPower(LS_full_speed);
                    RightLinearSlide.setPower(LS_full_speed);
                }
            }
            SzoneSlide_lastBump = gamepad2.x;


            //For Szone Slides code end=======================================================================

            //For Wrist code start======================================================================
            if (gamepad2.y) {
                int Wrist_sleeptimeValue = 250;
                if (WristPOS == WP3) {
                    if (WristClockwise == true) {
                        WristPOS = WP2;
                        Wrist.setPosition(WristPOS);
                        sleep(Wrist_sleeptimeValue);
                        WristClockwise = true;
                    }
                    else {
                        WristPOS = WP4;
                        Wrist.setPosition(WristPOS);
                        sleep(Wrist_sleeptimeValue);
                        WristClockwise = false;
                    }
                }
                else if(WristPOS == WP2) {
                    if (WristClockwise == true) {
                        WristPOS = WP1;
                        Wrist.setPosition(WristPOS);
                        sleep(Wrist_sleeptimeValue);
                        WristClockwise = false;
                    }
                    else {
                        WristPOS = WP3;
                        Wrist.setPosition(WristPOS);
                        sleep(Wrist_sleeptimeValue);
                        WristClockwise = false;
                    }
                }
                else if (WristPOS == WP1) {
                    if (WristClockwise == false) {
                        WristPOS = WP2;
                        Wrist.setPosition(WristPOS);
                        sleep(Wrist_sleeptimeValue);
                        WristClockwise = false;
                    }
                }
                else if (WristPOS == WP4) {
                    if (WristClockwise == true) {
                        WristPOS = WP3;
                        Wrist.setPosition(WristPOS);
                        sleep(Wrist_sleeptimeValue);
                        WristClockwise = true;
                    }
                    else {
                        WristPOS = WP5;
                        Wrist.setPosition(WristPOS);
                        sleep(Wrist_sleeptimeValue);
                        WristClockwise = true;
                    }
                }
                else if (WristPOS == WP5) {
                    if (WristClockwise == true) {
                        WristPOS = WP4;
                        Wrist.setPosition(WristPOS);
                        sleep(Wrist_sleeptimeValue);
                        WristClockwise = true;
                    }
                }
            }
            //For Wrist Slides code end=======================================================================

        }
    }
}
