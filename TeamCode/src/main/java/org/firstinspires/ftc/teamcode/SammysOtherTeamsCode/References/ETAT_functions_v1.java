package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//@Disabled
public class ETAT_functions_v1 {
    private final LinearOpMode myOpMode;
    public IMU imu;
    private final ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    // Hardware interface Objects
    private DcMotor FrontLeft = null;     //  control the left front drive wheel
    private DcMotor FrontRight = null;    //  control the right front drive wheel
    private DcMotor BackLeft = null;      //  control the left back drive wheel
    private DcMotor BackRight = null;     //  control the right back drive wheel

    //private DcMotor driveEncoder;
//    public DcMotor FrontLeftEncoder;        //  the left Axial (front/back) Odometry Module
//    public DcMotor FrontRightEncoder;       //  the right Axial (front/back) Odometry Module
//    public DcMotor BackMiddleEncoder;            //  the Lateral (left/right) Odometry Module
    public DcMotor LeftEncoder;        //  the left Axial (front/back) Odometry Module
    public DcMotor RightEncoder;       //  the right Axial (front/back) Odometry Module
    public DcMotor MiddleEncoder;            //  the Lateral (left/right) Odometry Module

    // For Drive Train
    double drive; // drive: left joystick y-axis
    double turn;  // turn: right joystick x-axis
    double strafe;  // strafe: left joystick x-axis
    double FLspeed, FRspeed, BLspeed, BRspeed;

    // For linear slides
    public DcMotor LeftLinearSlide;
    public DcMotor RightLinearSlide;
    public DcMotor LinearSlide;

    // For Claw Servo
    public Servo Claw_Servo = null;
    public final static double CLAW_HOME = 0.0;

    // For Active-Intake CR Servo
    public CRServo Active_Intake_CRServo = null;
    public final static double ACTIVE_INTAKE_IN = 1;
    public final static double ACTIVE_INTAKE_OUT = -1;
    public final static double ACTIVE_INTAKE_STOP = 0;

    // For Arm motor
    public DcMotor armMotor = null;

    // Robot Constructor
    public ETAT_functions_v1(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //Set all motor speed/power same
    public void set_myRobotAllMotorSpeedSame(double AllMotorSpeed){
        set_myRobotSpeed(AllMotorSpeed, AllMotorSpeed, AllMotorSpeed, AllMotorSpeed);
    }

    //Set each motor speed/power
    public void set_myRobotSpeed(double FLspeed, double FRspeed, double BLspeed, double BRspeed){
        FrontLeft.setPower(FLspeed);
        FrontRight.setPower(FRspeed);
        BackLeft.setPower(BLspeed);
        BackRight.setPower(BRspeed);
    }

    public void myRobot_HardwareMap(String frName, String brName, String flName, String blName,
//                                    String FrontLeftEncoderName, String FrontRightEncoderName,
                                    String LeftEncoderName, String RightEncoderName,
//                                    String BackMiddleEncoderName, String IMUname,
                                    String MiddleEncoderName, String IMUname,
                                    String LeftLinearSlideName, String RightLinearSlideName,
                                    String ClawServeName,
                                    String ActiveIntakeName,
                                    String ArmName){
        FrontRight = myOpMode.hardwareMap.dcMotor.get(frName);
        BackRight = myOpMode.hardwareMap.dcMotor.get(brName);
        FrontLeft = myOpMode.hardwareMap.dcMotor.get(flName);
        BackLeft = myOpMode.hardwareMap.dcMotor.get(blName);

//        FrontLeftEncoder = myOpMode.hardwareMap.dcMotor.get(FrontLeftEncoderName);
//        FrontRightEncoder = myOpMode.hardwareMap.dcMotor.get(FrontRightEncoderName);
//        BackMiddleEncoder = myOpMode.hardwareMap.dcMotor.get(BackMiddleEncoderName);
        LeftEncoder = myOpMode.hardwareMap.dcMotor.get(LeftEncoderName);
        RightEncoder = myOpMode.hardwareMap.dcMotor.get(RightEncoderName);
        MiddleEncoder = myOpMode.hardwareMap.dcMotor.get(MiddleEncoderName);

        // IMU Hardware initialize: Get the IMU from the configuration using hardwareMap. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = myOpMode.hardwareMap.get(IMU.class,IMUname);

//        // Linear slide Hardware initialize:
        LeftLinearSlide = myOpMode.hardwareMap.get(DcMotor.class, LeftLinearSlideName);
        RightLinearSlide = myOpMode.hardwareMap.get(DcMotor.class, RightLinearSlideName);

        // For Claw Servo Hardware initialize:
        Claw_Servo = myOpMode.hardwareMap.get(Servo.class,ClawServeName);

        // For Acive-Intake CRServo Hardware initialize:
        Active_Intake_CRServo = myOpMode.hardwareMap.get(CRServo.class,ActiveIntakeName);

        // For Arm motor:
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, ArmName);
    }

    public void myRobot_HardwareConfig(){
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        FrontLeftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontRightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackMiddleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        FrontLeftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontRightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackMiddleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MiddleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MiddleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //IMU sensor Hardware Configuration (Such as REV controlHub orientation)
//        imu.initialize(
//                new IMU.Parameters(new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT))
//        );
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );

        //IMU reset.
        imu.resetYaw();

        //Linear SLide
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        ((DcMotorEx) LeftLinearSlide).setCurrentAlert(5, CurrentUnit.AMPS);
//        ((DcMotorEx) RightLinearSlide).setCurrentAlert(5, CurrentUnit.AMPS);

        //Claw Servo
        Claw_Servo.setPosition(CLAW_HOME);

        // Active-Intake CRServo
        Active_Intake_CRServo.setPower(ACTIVE_INTAKE_STOP);

        // For Arm
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setPower(0);

        //Set all the motor speed/power to 0
        set_myRobotAllMotorSpeedSame(0.0);
    }

    //-------------------Telemetry Functions------------------------
    public void TelemetryInit(int refreshrate_ms){
        myOpMode.telemetry.addData("Status", "Initialized");
        myOpMode.telemetry.update();
        myOpMode.telemetry.setMsTransmissionInterval(refreshrate_ms);
    }
    public void TelemetryShowHeading(String Heading) {
        myOpMode.telemetry.addLine(Heading);
    }
    public void TelemetryShowHeadingData(String Heading, double Data) {
        myOpMode.telemetry.addData(Heading,Data);
    }
    public void TelemetryUpdate(int sleeptime) {
        myOpMode.telemetry.update();
        myOpMode.sleep(sleeptime);
    }


    //Drive Train code Start===============================================================
    public void Drivetrain(double Drive_button, double Turn_button, double Strafe_Button,
                           double Drive_Speed, double Turn_Speed, double Strafe_Speed){
        drive = (Drive_button * -1) * Drive_Speed;
        turn = (Turn_button) * Turn_Speed;
        strafe = (Strafe_Button) * Strafe_Speed;

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
        set_myRobotSpeed(FLspeed, FRspeed, BLspeed, BRspeed);
    }

    // Linear Slider functions
    public void setLinearSlide(String Direction, int TargetPosition, double Speed) {
        if (Direction == "UP") {
            LinearSlide.setTargetPosition(-TargetPosition);
            LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LinearSlide.setPower(Speed);
            while (myOpMode.opModeIsActive() && LinearSlide.isBusy()) {
                myOpMode.idle();
            }
            //LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LinearSlide.setPower(0);
        } else if (Direction == "DOWN") {
            LinearSlide.setTargetPosition(TargetPosition);
            LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LinearSlide.setPower(Speed);
            while (myOpMode.opModeIsActive() && LinearSlide.isBusy()) {
                myOpMode.idle();
            }
            //LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LinearSlide.setPower(0);
        } else {
            //LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LinearSlide.setPower(0);
        }
    }

    public void set_Left_Right_LinearSlide_v1(String Direction, int TargetPosition, double Speed) {
        if (Direction == "UP") {
            LeftLinearSlide.setTargetPosition(-TargetPosition);
            RightLinearSlide.setTargetPosition(-TargetPosition);

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftLinearSlide.setPower(Speed);
            RightLinearSlide.setPower(Speed);

            while (myOpMode.opModeIsActive() && LeftLinearSlide.isBusy() && RightLinearSlide.isBusy()) {
                myOpMode.idle();
            }
            //LeftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //RightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftLinearSlide.setPower(0);
            RightLinearSlide.setPower(0);

        } else if (Direction == "DOWN") {
            LeftLinearSlide.setTargetPosition(TargetPosition);
            RightLinearSlide.setTargetPosition(TargetPosition);

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftLinearSlide.setPower(Speed);
            RightLinearSlide.setPower(Speed);

            while (myOpMode.opModeIsActive() && LeftLinearSlide.isBusy() && RightLinearSlide.isBusy()) {
                myOpMode.idle();
            }
            //LeftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //RightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftLinearSlide.setPower(0);
            RightLinearSlide.setPower(0);
        } else {
            //LeftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //RightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftLinearSlide.setPower(0);
            RightLinearSlide.setPower(0);
        }
    }

    public void set_Left_Right_LinearSlide_v2(String Direction, int TargetPosition, double Speed) {
        if (Direction == "UP") {
            LeftLinearSlide.setTargetPosition(-TargetPosition);
            RightLinearSlide.setTargetPosition(-TargetPosition);

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftLinearSlide.setPower(Speed);
            RightLinearSlide.setPower(Speed);

            while (myOpMode.opModeIsActive() && LeftLinearSlide.isBusy() && RightLinearSlide.isBusy()) {
                myOpMode.idle();
            }
//            LeftLinearSlide.setPower(0);
//            RightLinearSlide.setPower(0);

        } else if (Direction == "DOWN") {
            LeftLinearSlide.setTargetPosition(TargetPosition);
            RightLinearSlide.setTargetPosition(TargetPosition);

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftLinearSlide.setPower(Speed);
            RightLinearSlide.setPower(Speed);

            while (myOpMode.opModeIsActive() && LeftLinearSlide.isBusy() && RightLinearSlide.isBusy()) {
                myOpMode.idle();
            }
//            LeftLinearSlide.setPower(0);
//            RightLinearSlide.setPower(0);
        } else {
            LeftLinearSlide.setPower(0);
            RightLinearSlide.setPower(0);
        }
    }

    // Logic did not work, --> Don't use the set_Left_Right_LinearSlide_Sync_v1 function.
    private static final double LIFT_SYNC_KP = 0.001;               //this value needs to be tuned
    private static final double LIFT_POSITION_TOLERANCE = 5;        //this value needs to be tuned
    public void set_Left_Right_LinearSlide_Sync_v1(String Direction, int TargetPosition, double Speed) {
        if (Direction == "UP") {
            LeftLinearSlide.setTargetPosition(-TargetPosition);
            RightLinearSlide.setTargetPosition(-TargetPosition);

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Motor1
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Motor2

            boolean isOnTarget = false;
            while (!isOnTarget)
            {
                double differentiatePower = (RightLinearSlide.getCurrentPosition() - LeftLinearSlide.getCurrentPosition())*LIFT_SYNC_KP;
                LeftLinearSlide.setPower(Range.clip(Speed + differentiatePower, -1.0, 1.0));
                RightLinearSlide.setPower(Range.clip(Speed - differentiatePower, -1.0, 1.0));
                isOnTarget = Math.abs(TargetPosition - LeftLinearSlide.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE &&
                        Math.abs(TargetPosition - RightLinearSlide.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;
                myOpMode.idle();
            }
            LeftLinearSlide.setPower(0.0);
            RightLinearSlide.setPower(0.0);
        }
        if (Direction == "DOWN") {
            LeftLinearSlide.setTargetPosition(TargetPosition);
            RightLinearSlide.setTargetPosition(TargetPosition);

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Motor1
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Motor2

            boolean isOnTarget = false;
            while (!isOnTarget)
            {
                double differentiatePower = (RightLinearSlide.getCurrentPosition() - LeftLinearSlide.getCurrentPosition())*LIFT_SYNC_KP;
                LeftLinearSlide.setPower(Range.clip(Speed + differentiatePower, -1.0, 1.0));
                RightLinearSlide.setPower(Range.clip(Speed - differentiatePower, -1.0, 1.0));
                isOnTarget = Math.abs(TargetPosition - LeftLinearSlide.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE &&
                        Math.abs(TargetPosition - RightLinearSlide.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;
                myOpMode.idle();
            }
            LeftLinearSlide.setPower(0.0);
            RightLinearSlide.setPower(0.0);
        }
    }

    // Logic worked. very stable.
    public void set_Left_Right_LinearSlide_Height_Sync_Control_v1(String Direction, int TargetPosition, double Speed) {
        if (Direction == "UP") {
            LeftLinearSlide.setTargetPosition(TargetPosition);
            RightLinearSlide.setTargetPosition(-TargetPosition);

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double Height_Control = LeftLinearSlide.getTargetPosition() - LeftLinearSlide.getCurrentPosition();
            double Sync_Control = LeftLinearSlide.getCurrentPosition() - RightLinearSlide.getCurrentPosition();

            double LeftLinearSlidePower = Height_Control;
            double RightLinearSlidePower = Height_Control + Sync_Control;

            double max_pwr = Math.max((Math.abs(LeftLinearSlidePower)),(Math.abs(RightLinearSlidePower)));
            if(max_pwr > 1){
                LeftLinearSlidePower = LeftLinearSlidePower / max_pwr;
                RightLinearSlidePower = RightLinearSlidePower / max_pwr;
            }
            LeftLinearSlide.setPower(LeftLinearSlidePower);
            RightLinearSlide.setPower(RightLinearSlidePower);
        }
        if (Direction == "DOWN") {
            LeftLinearSlide.setTargetPosition(TargetPosition);
            RightLinearSlide.setTargetPosition(TargetPosition);

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double Height_Control = LeftLinearSlide.getTargetPosition() - LeftLinearSlide.getCurrentPosition();
            double Sync_Control = LeftLinearSlide.getCurrentPosition() - RightLinearSlide.getCurrentPosition();

            double LeftLinearSlidePower = Height_Control;
            double RightLinearSlidePower = Height_Control + Sync_Control;

            double max_pwr = Math.max((Math.abs(LeftLinearSlidePower)),(Math.abs(RightLinearSlidePower)));
            if(max_pwr > 1){
                LeftLinearSlidePower = LeftLinearSlidePower / max_pwr;
                RightLinearSlidePower = RightLinearSlidePower / max_pwr;
            }
            LeftLinearSlide.setPower(LeftLinearSlidePower);
            RightLinearSlide.setPower(RightLinearSlidePower);
        }
    }


    public void set_Left_LinearSlide_v1(String Direction, int TargetPosition, double Speed) {
        if (Direction == "UP") {
            LeftLinearSlide.setTargetPosition(-TargetPosition);
            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftLinearSlide.setPower(Speed);

            while (myOpMode.opModeIsActive() && LeftLinearSlide.isBusy()) {
                myOpMode.idle();
            }
            LeftLinearSlide.setPower(0);

        } else if (Direction == "DOWN") {
            LeftLinearSlide.setTargetPosition(TargetPosition);
            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftLinearSlide.setPower(Speed);

            while (myOpMode.opModeIsActive() && LeftLinearSlide.isBusy()) {
                myOpMode.idle();
            }
            LeftLinearSlide.setPower(0);
        } else {
            LeftLinearSlide.setPower(0);
        }
    }

    public void setVelocity_Left_Right_LinearSlide_ver1(String Direction, int TargetPosition, double Speed) {
        double fast_ls_speed = abs(Speed / 100) * 80;
        double slow_ls_speed = abs(Speed - fast_ls_speed);

        if (Direction == "UP") {
            int fast_LS_Position = 80;
            int LS_Position_80 = (abs(LeftLinearSlide.getCurrentPosition() - TargetPosition))/100 * fast_LS_Position;

            LeftLinearSlide.setTargetPosition((int) (-LS_Position_80));
            RightLinearSlide.setTargetPosition((int) (-LS_Position_80));

            ((DcMotorEx) LeftLinearSlide).setVelocity(fast_ls_speed);  //2100
            ((DcMotorEx) RightLinearSlide).setVelocity(fast_ls_speed);  //2100

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (LeftLinearSlide.isBusy() && RightLinearSlide.isBusy()){
                myOpMode.idle();
            }

            LeftLinearSlide.setTargetPosition((int) (-TargetPosition));
            RightLinearSlide.setTargetPosition((int) (-TargetPosition));

            ((DcMotorEx) LeftLinearSlide).setVelocity(slow_ls_speed);  //2100
            ((DcMotorEx) RightLinearSlide).setVelocity(slow_ls_speed);  //2100

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (LeftLinearSlide.isBusy() && RightLinearSlide.isBusy()){
                myOpMode.idle();
            }
//            // Sleep mode.????
//            ((DcMotorEx) LeftLinearSlide).setVelocity(0);  //2100
//            ((DcMotorEx) RightLinearSlide).setVelocity(0);  //2100

        } else if (Direction == "DOWN") {
            int fast_LS_Position = 20;
            int LS_Position_80 = (abs(LeftLinearSlide.getCurrentPosition() - TargetPosition))/100 * fast_LS_Position;

            LeftLinearSlide.setTargetPosition((int) (-LS_Position_80));
            RightLinearSlide.setTargetPosition((int) (-LS_Position_80));

            ((DcMotorEx) LeftLinearSlide).setVelocity(fast_ls_speed);  //2100
            ((DcMotorEx) RightLinearSlide).setVelocity(fast_ls_speed);  //2100

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (LeftLinearSlide.isBusy() && RightLinearSlide.isBusy()){
                myOpMode.idle();
            }

            LeftLinearSlide.setTargetPosition((int) (-TargetPosition));
            RightLinearSlide.setTargetPosition((int) (-TargetPosition));

            ((DcMotorEx) LeftLinearSlide).setVelocity(slow_ls_speed);  //2100
            ((DcMotorEx) RightLinearSlide).setVelocity(slow_ls_speed);  //2100

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (LeftLinearSlide.isBusy() && RightLinearSlide.isBusy()){
                myOpMode.idle();
            }
//            // Sleep mode.????
//            ((DcMotorEx) LeftLinearSlide).setVelocity(0);  //2100
//            ((DcMotorEx) RightLinearSlide).setVelocity(0);  //2100
        } else {
            ((DcMotorEx) LeftLinearSlide).setVelocity(0);  //2100
            ((DcMotorEx) RightLinearSlide).setVelocity(0);  //2100
        }

        if (((DcMotorEx) LeftLinearSlide).isOverCurrent()){
            myOpMode.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!: LeftLinearSlide");
            myOpMode.telemetry.update();
        }
        if (((DcMotorEx) RightLinearSlide).isOverCurrent()){
            myOpMode.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!: RightLinearSlide");
            myOpMode.telemetry.update();
        }
    }

    public void setVelocity_Left_Right_LinearSlide_ver2(String Direction, int TargetPosition, double Speed) {
        if (Direction == "UP") {
            LeftLinearSlide.setTargetPosition((int) (TargetPosition));
            RightLinearSlide.setTargetPosition((int) (TargetPosition));

            ((DcMotorEx) LeftLinearSlide).setVelocity(Speed);  //2100
            ((DcMotorEx) RightLinearSlide).setVelocity(Speed);  //2100

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (Direction == "DOWN") {
            LeftLinearSlide.setTargetPosition((int) (TargetPosition));
            RightLinearSlide.setTargetPosition((int) (TargetPosition));

            ((DcMotorEx) LeftLinearSlide).setVelocity(Speed);  //2100
            ((DcMotorEx) RightLinearSlide).setVelocity(Speed);  //2100

            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (((DcMotorEx) LeftLinearSlide).isOverCurrent()){
            myOpMode.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!: LeftLinearSlide");
            myOpMode.telemetry.update();
        }
        if (((DcMotorEx) RightLinearSlide).isOverCurrent()){
            myOpMode.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!: RightLinearSlide");
            myOpMode.telemetry.update();
        }
    }



    public void setLinearSlidePosition(int TargetPosition, double Power) {
        LeftLinearSlide.setTargetPosition(TargetPosition);
        RightLinearSlide.setTargetPosition(TargetPosition);

        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftLinearSlide.setPower(Power);
        RightLinearSlide.setPower(Power);
    }

    public void setLinearSlidePower(double power) {
        // If the motor is busy, don't allow this function to command the motor.
        // However, if the motor is busy, update telemetry.
        if( LeftLinearSlide.isBusy() && RightLinearSlide.isBusy())
        {
            // This function always gets called, so it makes sense for the telemetry
            // to be here whenever this function isn't commanding the arm.
            myOpMode.telemetry.addData("Current POS","LLS: %.0f, RLS: %.0f", LeftLinearSlide.getCurrentPosition(), RightLinearSlide.getCurrentPosition());
            myOpMode.telemetry.addData("Target POS", "LLS: %.0f, RLS: %.0f", LeftLinearSlide.getTargetPosition(),RightLinearSlide.getTargetPosition());
            myOpMode.telemetry.update();
        }
        else
        {
            // Only command the Linear slide if the firmware isn't currently using it.
            LeftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            LeftLinearSlide.setPower(power);
            RightLinearSlide.setPower(power);
        }
    }
    //----------------------------------------------------
    // Claw functions

    //----------------------------------------------------
    //----------------------------------------------------
    // Active-Intake CRServo functions
    public void setActiveIntake(String Operation) {
        // Operation : IN  or OUT or STOP
        if (myOpMode.opModeIsActive() && Operation == "IN")
            Active_Intake_CRServo.setPower(ACTIVE_INTAKE_IN);

        if (myOpMode.opModeIsActive() && Operation == "OUT")
            Active_Intake_CRServo.setPower(ACTIVE_INTAKE_OUT);

        if (myOpMode.opModeIsActive() && Operation == "STOP")
            Active_Intake_CRServo.setPower(ACTIVE_INTAKE_STOP);
    }

    //----------------------------------------------------
    // Arm functions
    public void setArm_version1(String Direction, int Position, double Speed) {
        double fast_arm_speed = abs(Speed / 100) * 80;
        double slow_arm_speed = abs(Speed - fast_arm_speed);

        if (Direction == "HOME") {
            int fast_Arm_Position = 20;
            int Arm_Position_80 = (abs(armMotor.getCurrentPosition() - Position))/100 * fast_Arm_Position;
            armMotor.setTargetPosition((int) (Arm_Position_80));
            ((DcMotorEx) armMotor).setVelocity(fast_arm_speed);  //2100
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (armMotor.isBusy()){
                myOpMode.idle();
            }

            armMotor.setTargetPosition((int) (Position));
            ((DcMotorEx) armMotor).setVelocity(slow_arm_speed);  //2100
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (Direction == "PICKUP") {
            int fast_Arm_Position = 80;
            int Arm_Position_80 = (abs(armMotor.getCurrentPosition() - Position))/100 * fast_Arm_Position;
            armMotor.setTargetPosition((int) (Arm_Position_80));
            ((DcMotorEx) armMotor).setVelocity(fast_arm_speed);  //2100
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (armMotor.isBusy()){
                myOpMode.idle();
            }

            armMotor.setTargetPosition((int) (Position));
            ((DcMotorEx) armMotor).setVelocity(slow_arm_speed);  //2100
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (Direction == "DROP_or_CLIP") {
            int fast_Arm_Position = 80;
            if (armMotor.getCurrentPosition() < 25){
                int Arm_Position_80 = ((abs(armMotor.getCurrentPosition() - Position))/100 * fast_Arm_Position);
                armMotor.setTargetPosition((int) (Arm_Position_80));
            }
            if (armMotor.getCurrentPosition() > 750 ) {
                int Arm_Position_80 = ((abs(armMotor.getCurrentPosition() - Position))/100 * fast_Arm_Position) + Position;
                armMotor.setTargetPosition((int) (Arm_Position_80));
            }

            ((DcMotorEx) armMotor).setVelocity(fast_arm_speed);  //2100
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (armMotor.isBusy()){
                myOpMode.idle();
            }

            armMotor.setTargetPosition((int) (Position));
            ((DcMotorEx) armMotor).setVelocity(slow_arm_speed);  //2100
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (((DcMotorEx) armMotor).isOverCurrent()){
            myOpMode.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!: LeftLinearSlide");
            myOpMode.telemetry.update();
        }

        /* send telemetry to the driver of the arm's current position and target position */
        myOpMode.telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        myOpMode.telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        myOpMode.telemetry.update();
    }

    public void setArm_version2(String Direction, int TargetPosition, double Speed) {
        final double ARM_COLLAPSED_INTO_ROBOT  = 0;
        double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;

        if (Direction == "HOME") {
            armMotor.setTargetPosition((int) (TargetPosition));
            ((DcMotorEx) armMotor).setVelocity(Speed);  // 3
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

//        if (Direction == "PICKUP") {
//            armPosition= Math.abs((TargetPosition/100)* 95);
//            armMotor.setTargetPosition((int) (armPosition));
//            ((DcMotorEx) armMotor).setVelocity(Speed);  //2100
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (armMotor.isBusy()){
//                myOpMode.idle();
//            }
//
//            armMotor.setTargetPosition((int) (TargetPosition));
//            ((DcMotorEx) armMotor).setVelocity(Speed/2);  //2100
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//////            while (armMotor.isBusy()){
//////                myOpMode.idle();
//////            }
//        }

        if (Direction == "PICKUP") {
//            armPosition= Math.abs((Position/100)* 95);
            armMotor.setTargetPosition((int) (TargetPosition));
            ((DcMotorEx) armMotor).setVelocity(Speed);  //2100
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//        else if (Direction == "DROP_or_CLIP") {
//            armMotor.setTargetPosition((int) (TargetPosition));
//            ((DcMotorEx) armMotor).setVelocity(Speed);  //2100
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

        if (Direction == "DROP_or_CLIP") {
            armMotor.setTargetPosition((int) (TargetPosition));
            ((DcMotorEx) armMotor).setVelocity(Speed);  //2100
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        if (((DcMotorEx) armMotor).isOverCurrent()){
            myOpMode.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!: LeftLinearSlide");
            myOpMode.telemetry.update();
        }

        /* send telemetry to the driver of the arm's current position and target position */
        myOpMode.telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        myOpMode.telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        myOpMode.telemetry.update();
    }

    public void setArm(String Direction, int Position, double Speed) {
        if (Direction == "HOME") {
            armMotor.setTargetPosition(Position);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(Speed);
            while (myOpMode.opModeIsActive() && armMotor.isBusy()) {
                myOpMode.idle();
            }
            //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(0);
        } else if (Direction == "UP") {
            armMotor.setTargetPosition(Position);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(Speed);
            while (myOpMode.opModeIsActive() && armMotor.isBusy()) {
                myOpMode.idle();
            }
            //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            armMotor.setPower(0);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (Direction == "DOWN") {
            armMotor.setTargetPosition(Position);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(Speed);
            while (myOpMode.opModeIsActive() && armMotor.isBusy()) {
                myOpMode.idle();
            }
            //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            armMotor.setPower(0);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } else {
            //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            armMotor.setPower(0);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    //----------------------------------------------------

    //-------------------IMU Functions------------------------
    public double get_myRobotYamAngle (){
        double myRobotYamAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return myRobotYamAngle;
    }

    public double get_myRobotPitchAngle (){
        double myRobotPitchAngle = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        return myRobotPitchAngle;
    }
    public double get_myRobotRollAngle (){
        double myRobotRollAngle = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
        return myRobotRollAngle;
    }

    //-------------------Rotate Function----------------------
    public void Rotate(double speed){
        set_myRobotSpeed(-speed, speed, -speed, speed);
    }

    //-------------------TURN Functions-----------------------
//    public void turn90(double targetDegree, double speed)
//    {
//        set_myRobotSpeed(speed,-speed,speed,-speed);
//        // Continue until robot yaws right by target degrees or stop is pressed on Driver Station.
//        double Yaw_Angle = get_myRobotYamAngle();
//        if ((Yaw_Angle >= 90) && (Yaw_Angle <= 45)){
//            speed = 0.75;
//
//        }
//
//        while ( !(Yaw_Angle >= targetDegree || myOpMode.isStopRequested()) ) {
//            // Update Yaw-Angle variable with current yaw.
//            Yaw_Angle = get_myRobotYamAngle();
//            // Report yaw orientation to Driver Station.
//            TelemetryShowHeadingData("Yaw value", Yaw_Angle);
//            TelemetryUpdate(100);
//        }
//        // Target reached. Turn off motors
//        set_myRobotAllMotorSpeedSame(0);
//    }

    public void turnRight(double targetDegree, double speed)
    {
        set_myRobotSpeed(speed,-speed,speed,-speed);
        // Continue until robot yaws right by target degrees or stop is pressed on Driver Station.
        double Yaw_Angle = get_myRobotYamAngle();
        while ( !(Yaw_Angle >= targetDegree || myOpMode.isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = get_myRobotYamAngle();
            // Report yaw orientation to Driver Station.
            TelemetryShowHeadingData("Yaw value", Yaw_Angle);
            TelemetryUpdate(100);
        }
        // Target reached. Turn off motors
        set_myRobotAllMotorSpeedSame(0);
    }

    public void turnLeft(double targetDegree, double speed)
    {
        set_myRobotSpeed(-speed,speed,-speed,speed);
        // Continue until robot yaws left by target degrees or stop is pressed on Driver Station.
        double Yaw_Angle = get_myRobotYamAngle();
        while ( !(Yaw_Angle >= targetDegree || myOpMode.isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = get_myRobotYamAngle();
            // Report yaw orientation to Driver Station.
            TelemetryShowHeadingData("Yaw value", Yaw_Angle);
            TelemetryUpdate(100);
        }
        // Target reached. Turn off motors
        set_myRobotAllMotorSpeedSame(0);
    }

    // RelativeTurn : From where ever robot is facing turn from that point.
    public void RelativeTurn(double TargetDegrees){
        imu.resetYaw();
//        double TurnError = TargetDegrees;
        double TurnError = TargetDegrees - get_myRobotYamAngle();
        while (myOpMode.opModeIsActive() && (abs(TurnError) > 2)){
            double RobotSpeed = (TurnError < 0 ? -0.3 : 0.3);
            set_myRobotSpeed(-RobotSpeed, RobotSpeed, -RobotSpeed, RobotSpeed);
            TurnError = TargetDegrees - get_myRobotYamAngle();
            TelemetryShowHeadingData("How Far Away From Traget Degrees : ", TurnError);
            TelemetryUpdate(0);
        }
        set_myRobotAllMotorSpeedSame(0);
    }

    //AbsoluteTurn To target Degree/Angle based on initial robot heading
    public void AbsoluteTurnTo(double TargetDegrees){
        double YamAngle_Current = get_myRobotYamAngle();
        double TurnError = TargetDegrees - YamAngle_Current;

        if (TurnError > 180) { TurnError = TurnError - 360;}
        else if (TurnError < -180) {TurnError = TurnError + 360;}

        RelativeTurn(TurnError);
    }

    public double getAbsoluteAngle(){
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        myOpMode.telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            set_myRobotSpeed(-motorPower/2, motorPower/2, -motorPower/2, motorPower/2);

            myOpMode.telemetry.addData("Current Angle", getAbsoluteAngle());
            myOpMode.telemetry.addData("Target Angle", targetAngle);
            myOpMode.telemetry.addData("Slope", pid.getLastSlope());
            myOpMode.telemetry.addData("Power", motorPower);
            myOpMode.telemetry.update();
        }
        set_myRobotAllMotorSpeedSame(0);
    }

    // Single use per object
    public class TurnPIDController {
        private final double kP;
        private final double kI;
        private final double kD;
        private final ElapsedTime timer = new ElapsedTime();
        private final double targetAngle;
        private double lastError = 0;
        private double accumulatedError = 0;
        private double lastTime = -1;
        private double lastSlope = 0;

        public TurnPIDController(double target, double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
            targetAngle = target;
        }

        public double update(double currentAngle) {
            // TODO: make sure angles are within bounds and are in same format (e.g., 0 <= | angle | <= 180)
            //   and ensure direction is correct

            // P
            double error = targetAngle - currentAngle;
            error %= 360;
            error += 360;
            error %= 360;
            if (error > 180) {
                error -= 360;
            }

            // I
            accumulatedError *= Math.signum(error);
            accumulatedError += error;
            if (abs(error) < 2) {
                accumulatedError = 0;
            }

            // D
            double slope = 0;
            if (lastTime > 0) {
                slope = (error - lastError) / (timer.milliseconds() - lastTime);
            }
            lastSlope = slope;
            lastError = error;
            lastTime = timer.milliseconds();

            double motorPower = 0.1 * Math.signum(error)
//                    + 0.9 * Math.tanh(kP * error + kI * accumulatedError - kD * slope);
                    + 0.9 * Math.tanh(kP * error + kI * accumulatedError + kD * slope);

            return motorPower;
        }

        public double getLastSlope() {
            return lastSlope;
        }
    }

    // another method PID
    private double integralSum = 0;
    private final double Kp = 2.0;
    private final double Ki = 0.0;
    private final double Kd = 0.0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    public double TurnPIDControl_v2(double target, double current_state) {
        double error = angleWrap(target - current_state);
        myOpMode.telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    //----------------------------------------------------


}


