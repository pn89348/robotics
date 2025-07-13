package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;
import static java.lang.Math.abs;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "ROBOT3_T3_TeleOp_v1", group = "ETAT")
//@Disabled
public class ROBOT3_T3_TeleOp_v1 extends LinearOpMode {
    String flName = "fl", frName = "fr", blName = "bl", brName = "br";
    String LeftLinearSlideName = "lsl", RightLinearSlideName = "lsr";
    String SpecimenClawName = "specimenclaw";
    String SpecimenWristName = "specimenwrist";
    String SpecimenElbowName = "specimenelbow";
    String SpecimenArmLeftName = "specimenarmleft";
    String SpecimenArmRightName = "specimenarmright";
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
    public final static double SPECIMEN_CLAW_OPEN = 0.6;
    public final static double SPECIMEN_CLAW_OPEN_first = 0.6;
    public final static double SPECIMEN_CLAW_OPEN_S = 0.6;
    private boolean Specimen_Claw_Open = true;
    private boolean Specimen_Claw_lastBump = false;
    // For Specimen_Wrist
    public Servo Specimen_Wrist = null;
    public final static double SPECIMEN_WRIST_HOME = 0.0;
    public final static double SPECIMEN_WRIST_CLOSE = 0.0;
    public final static double SPECIMEN_WRIST_OPEN = 0.6;
    public final static double SPECIMEN_WRIST_OPEN_first = 0.6;
    public final static double SPECIMEN_WRIST_OPEN_S = 0.6;
    private boolean Specimen_Wrist_Open = true;
    private boolean Specimen_Wrist_lastBump = false;
    // For Specimen_Elbow
    public Servo Specimen_Elbow = null;
    public final static double SPECIMEN_ELBOW_HOME = 0.0;
    public final static double SPECIMEN_ELBOW_CLOSE = 0.0;
    public final static double SPECIMEN_ELBOW_OPEN = 0.6;
    public final static double SPECIMEN_ELBOW_OPEN_first = 0.6;
    public final static double SPECIMEN_ELBOW_OPEN_S = 0.6;
    private boolean Specimen_Elbow_Open = true;
    private boolean Specimen_Elbow_lastBump = false;
    // For Specimen_Arm_Left
    public Servo Specimen_Arm_Left = null;
    public final static double SPECIMEN_ARM_LEFT_HOME = 0.0;
    public final static double SPECIMEN_ARM_LEFT_CLOSE = 0.0;
    public final static double SPECIMEN_ARM_LEFT_OPEN = 0.6;
    public final static double SPECIMEN_ARM_LEFT_OPEN_first = 0.6;
    public final static double SPECIMEN_ARM_LEFT_OPEN_S = 0.6;
    private boolean Specimen_Arm_Left_Open = true;
    private boolean Specimen_Arm_Left_lastBump = false;
    // For Specimen_Arm
    public Servo Specimen_Arm_Right = null;
    public final static double SPECIMEN_ARM_RIGHT_HOME = 0.0;
    public final static double SPECIMEN_ARM_RIGHT_CLOSE = 0.0;
    public final static double SPECIMEN_ARM_RIGHT_OPEN = 0.6;
    public final static double SPECIMEN_ARM_RIGHT_OPEN_first = 0.6;
    public final static double SPECIMEN_ARM_RIGHT_OPEN_S = 0.6;
    private boolean Specimen_Arm_Right_Open = true;
    private boolean Specimen_Arm_Right_lastBump = false;
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
        double strafe_speed = 0.75; //0.5  -- 0.75
        double FLspeed, FRspeed, BLspeed, BRspeed;
        //Robot Hardware Mapping:
        FrontLeft = hardwareMap.get(DcMotor.class, flName);
        FrontRight = hardwareMap.get(DcMotor.class, frName);
        BackLeft = hardwareMap.get(DcMotor.class, blName);
        BackRight = hardwareMap.get(DcMotor.class, brName);
        Specimen_Claw = hardwareMap.get(Servo.class, SpecimenClawName);
        Specimen_Wrist = hardwareMap.get(Servo.class, SpecimenWristName);
        Specimen_Elbow = hardwareMap.get(Servo.class, SpecimenElbowName);
        Specimen_Arm_Left = hardwareMap.get(Servo.class, SpecimenArmLeftName);
        Specimen_Arm_Right = hardwareMap.get(Servo.class, SpecimenArmRightName);
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
        // Initial conditions:
        //For Drive Train
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        //For SPECIMEN CLAW
        Specimen_Claw.setPosition(0.20);  //0.6
        //For SPECIMEN WRIST
        Specimen_Wrist.setPosition(0.025); //0.025
        //For SPECIMEN ELBOW
        Specimen_Elbow.setPosition(0.95); //0.5
//        Specimen_Elbow.setPosition(0.09); //0.5
        //For SPECIMEN ARM
        Specimen_Arm_Left.setPosition(0.95); //0.85
        Specimen_Arm_Right.setPosition(0.95); //0.85
//        Specimen_Arm_Left.setPosition(0.05); //0.85
//        Specimen_Arm_Right.setPosition(0.05); //0.85
        telemetry.addLine("ROBOT3-TeleOp: Press START > button");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            //Drive Train code Start===============================================================
//            drive = (gamepad1.left_stick_y * -1) * drive_speed;
            drive = (gamepad1.left_stick_y * 1) * drive_speed;
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
            //For Specimen_Claw code start===================================================================
            if (gamepad2.right_bumper && !Specimen_Claw_lastBump) {
                Specimen_Claw_Open = !Specimen_Claw_Open;
                if (Specimen_Claw_Open) {
                    Specimen_Claw.setPosition(0.25);
                } else {
                    Specimen_Claw.setPosition(0.0);
                }
            }
            Specimen_Claw_lastBump = gamepad2.right_bumper;
            // For Specimen_Claw code end====================================================================
            //For Specimen_wrist code start===================================================================
            if (gamepad2.x && !Specimen_Wrist_lastBump) {
                Specimen_Wrist_Open = !Specimen_Wrist_Open;
                if (Specimen_Wrist_Open) {
                    Specimen_Wrist.setPosition(0.78);
                } else {
                    Specimen_Wrist.setPosition(0.025);
                }
            }
            Specimen_Wrist_lastBump = gamepad2.x;
            // For Specimen_wrist code end====================================================================
            //For Specimen_Elbow code start===================================================================
            if (gamepad2.y && !Specimen_Elbow_lastBump) {
                Specimen_Elbow_Open = !Specimen_Elbow_Open;
                if (Specimen_Elbow_Open) {
                    Specimen_Elbow.setPosition(0.5); //0.55
                } else {
                    Specimen_Elbow.setPosition(0.7); //0.7
                }
            }
            Specimen_Elbow_lastBump = gamepad2.y;
            // For Specimen_wrist code end====================================================================
            //For Specimen_Arm code start===================================================================
//            if (gamepad2.b && !Specimen_Arm_lastBump ) {
//                Specimen_Arm_Open = !Specimen_Arm_Open;
//                if (Specimen_Arm_Open) {
//                    Specimen_Arm.setPosition(0.2);
//                } else {
//                    Specimen_Arm.setPosition(0.9);
//                }
//            }
//            Specimen_Arm_lastBump = gamepad2.b;
            if (gamepad2.b && !Specimen_Arm_Left_lastBump && !Specimen_Arm_Right_lastBump) {
                Specimen_Arm_Left_Open = !Specimen_Arm_Left_Open;
                Specimen_Arm_Right_Open = !Specimen_Arm_Right_Open;
                if (Specimen_Arm_Left_Open && Specimen_Arm_Right_Open) {
//                    Specimen_Arm_Left.setPosition(0.2);
//                    Specimen_Arm_Right.setPosition(0.2);
//                    Specimen_Elbow.setPosition(0.5); //0.55
                    Specimen_Arm_Left.setPosition(0.05); //0.85
                    Specimen_Arm_Right.setPosition(0.05); //0.85
                    Specimen_Elbow.setPosition(0.09); //0.09
                } else {
                    Specimen_Arm_Left.setPosition(0.85); //0.9
                    Specimen_Arm_Right.setPosition(0.85); //0.9
                    Specimen_Elbow.setPosition(0.53); //0.5
                }
            }
            Specimen_Arm_Left_lastBump = gamepad2.b;
            Specimen_Arm_Right_lastBump = gamepad2.b;
            // For Specimen_wrist code end====================================================================
        }
    }
}