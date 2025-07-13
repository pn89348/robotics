package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;//package General.References;
//



////Please dont edit this one. Only use it for reference
////Use v7 instead to edit






//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
////@Disabled
//@Autonomous(name = "ETAT_AUTO_v6_RIGHT_new", group = "ETAT")
////@TeleOp(name = "ETAT_AUTO_v6_RIGHT", group = "ETAT")
//public class ETAT_AUTO_v6_RIGHT_new extends LinearOpMode {
//    //Hardware Map Names
//    String flName = "fl", frName = "fr", blName = "bl", brName = "br";
//    String LeftLinearSlideName = "lsl", RightLinearSlideName = "lsr";
//    String ClawServoName = "claw";
//    String ArmName = "arm";
//    String verticalRightEncoderName = frName, verticalLeftEncoderName = flName, horizontalEncoderName = blName;
//
//    // Define or Declare Hardware here...
//    public DcMotor FrontLeft = null;
//    public DcMotor FrontRight = null;
//    public DcMotor BackLeft = null;
//    public DcMotor BackRight = null;
//
//    // Odometry Wheels
//    public DcMotor verticalLeft = null;
//    public DcMotor verticalRight = null;
//    public DcMotor horizontal = null;
//
//    //    final double COUNTS_PER_INCH = 307.699557;
//    static final double     COUNTS_PER_MOTOR_REV    = 2000 ;   // Odometry Encoder Countable Events per Revolution or Encoder Resolution
//    // https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/
//    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     //1.0 No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 1.88976 ;     //  4.0 For figuring circumference
//    // Odometry wheels diameter = 48mm wheels: Convert mm to inchs (48mm is 1.88976 inches)
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    // IMU
//    //IMU Sensor
//    public final IMU imu         = null;      // Control/Expansion Hub IMU
//    Orientation myRobotOrientation;
//    YawPitchRollAngles robotOrientation;
//    double YAW_angle;
//
//    // For linear slides
//    public DcMotor LeftLinearSlide = null;
//    public DcMotor RightLinearSlide = null;
//
//    final double LS_TICKS_PER_MM = 537.7 / 120.0;
//    final double LS_COLLAPSED = 0 * LS_TICKS_PER_MM;
//    final double LS_SCORING_IN_LOW_BASKET = 0 * LS_TICKS_PER_MM;
//    final double LS_SCORING_IN_HIGH_BASKET = 483 * LS_TICKS_PER_MM;  // 480
//    final double LS_MAX_MOTOR_CURRENT = 2.0;
//
//    double LS_full_speed = 1.0;   //0.8 or 1.0
//    double LS_half_speed = LS_full_speed/3;   // div 2
//    int LS_TargetPosition = (int)LS_COLLAPSED;
//    boolean LS_motorRunning = false;
//    double LS_RESET_POSITION = 0;
//    double LS_TOLERANCE = 2.0;
//
////    //FOR ARM1 Motor: we are using 30RPM motor
////    final double ARM_TICKS_PER_DEGREE =
////            28 // number of encoder ticks per rotation of the bare motor
////                    * 188.0 / 1.0 // This is the exact gear ratio of the 30RPM 188:1 Yellow Jacket gearbox
////                    * 24.0 / 24.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
////                    * 1/360.0; // we want ticks per degree, not per rotation
//
//    //FOR ARM2 Motor: We are using 117RPM motor
//    final double ARM_TICKS_PER_DEGREE =
//            28 // number of encoder ticks per rotation of the bare motor
//                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
//                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
//                    * 1/360.0; // we want ticks per degree, not per rotation
//
//    public DcMotor armMotor = null;
//
////    final double ARM_COLLAPSED_INTO_ROBOT  = 0 * ARM_TICKS_PER_DEGREE;
//////    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
//////    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
////    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
////    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
//////    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
////    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
////
////    final double ARM_ATTACH_HANGING_HOOK   = 166 * ARM_TICKS_PER_DEGREE;
////    final double ARM_CLEAR_BARRIER         = 235 * ARM_TICKS_PER_DEGREE;
////    final double ARM_PICKUP               = 255 * ARM_TICKS_PER_DEGREE;
////    final double ARM_CLOSER_TO_REST       = 60  * ARM_TICKS_PER_DEGREE;
////
////    /* A number in degrees that the triggers can adjust the arm position by */
////    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; //15
//
//    final double ARM_COLLAPSED_INTO_ROBOT  = 0 * ARM_TICKS_PER_DEGREE;
//    //    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
////    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
//    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
//    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
//    //    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
//    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
//
//    final double ARM_ATTACH_HANGING_HOOK   = 175 * ARM_TICKS_PER_DEGREE;  //166
//    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;  //235
//    final double ARM_PICKUP               = 252 * ARM_TICKS_PER_DEGREE;
//    final double ARM_CLOSER_TO_REST       = 60  * ARM_TICKS_PER_DEGREE;
//    final double ARM_CLIP                 = 210 * ARM_TICKS_PER_DEGREE;
//
//    /* A number in degrees that the triggers can adjust the arm position by */
//    final double FUDGE_FACTOR = 25 * ARM_TICKS_PER_DEGREE; //15
//
//    double armTargetPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
//    double armPositionFudgeFactor;
//
//    double arm_full_speed = 0.6;
//    double arm_half_speed = arm_full_speed/2;
//    boolean arm_motorRunning = false;
//    double ARM_RESET_POSITION = 0;
//    double ARM_TOLERANCE = 2.0;
//
//    // For Claw
//    public Servo Claw_Servo = null;
//    public final static double CLAW_HOME = 0.0;
//    public final static double CLAW_MIN_RANGE = 0.0;
//    public final static double CLAW_MAX_RANGE = 0.5;
//    public final static double CLAW_CLOSE = 0.0;
//    public final static double CLAW_OPEN = 0.25;
//    public final static double CLAW_OPEN_S = 0.17;
//    double clawOperation = CLAW_HOME;
//
//    // Odometry
//    OdometryGlobalCoordinatePosition globalPositionUpdate;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //Initialize Telemetry.
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        double drive; // drive: left joystick y-axis
//        double turn;  // turn: right joystick x-axis
//        double strafe;  // strafe: left joystick x-axis
//        double drive_speed = 0.8;
//        double turn_speed = 0.5;
//        double strafe_speed = 0.5;
//        double FLspeed, FRspeed, BLspeed, BRspeed;
//
//        //Robot Hardware Mapping:
//        FrontLeft = hardwareMap.get(DcMotor.class, flName);
//        FrontRight = hardwareMap.get(DcMotor.class, frName);
//        BackLeft = hardwareMap.get(DcMotor.class, blName);
//        BackRight = hardwareMap.get(DcMotor.class, brName);
//        LeftLinearSlide = hardwareMap.dcMotor.get(LeftLinearSlideName);
//        RightLinearSlide = hardwareMap.dcMotor.get(RightLinearSlideName);
//        Claw_Servo = hardwareMap.get(Servo.class,ClawServoName);
//        armMotor = hardwareMap.dcMotor.get(ArmName);
//        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
//        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
//        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);
//
//        // Robot Hardware Configuration:
//        //For Drive Train
//        // Reverse Left side motors.
//        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        //STOP_AND_RESET_ENCODER
//        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //BRAKE-- need to check...
////        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        //SET Run without encoder
//        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        //For Linear SLide
//        RightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // For Arm
////        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Odometry
//        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // IMU
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//
//        myRobotOrientation = imu.getRobotOrientation(
//                AxesReference.INTRINSIC,
//                AxesOrder.ZYX,
//                AngleUnit.DEGREES
//        );
//        imu.resetYaw();
//
//        // Initial conditions:
//        //For Drive Train
//        FrontLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackLeft.setPower(0);
//        BackRight.setPower(0);
//
//        //For Linear SLide
//        LS_motorRunning = false;
//        LS_TargetPosition = (int) (LS_COLLAPSED);
//
//        LeftLinearSlide.setTargetPosition((int) (LS_COLLAPSED));
//        RightLinearSlide.setTargetPosition((int) (LS_COLLAPSED));
//        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //For Claw
//        Claw_Servo.setPosition(CLAW_OPEN);
//
//        // For Arm
//        armMotor.setTargetPosition((int)(ARM_COLLAPSED_INTO_ROBOT));
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setPower(0);
//
//        telemetry.addLine("Initialization & Configuration : Press START > button ");
//        telemetry.update();
//
//        waitForStart();
//
//        // For Odometry
//        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
//        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
//        Thread positionThread = new Thread(globalPositionUpdate);
//        positionThread.start();
//
//        globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseLeftEncoder();
////        globalPositionUpdate.reverseNormalEncoder();
//
//        if (Math.abs(armMotor.getCurrentPosition() - armTargetPosition) < ARM_TOLERANCE && armTargetPosition!= ARM_RESET_POSITION) {
//            armMotor.setPower(arm_half_speed);
//            arm_motorRunning=false;
//        }
//
//        if((armMotor.getCurrentPosition()>=ARM_RESET_POSITION && armMotor.getCurrentPosition()<=10) && armTargetPosition==ARM_RESET_POSITION ) {
//            armMotor.setPower(0);
//            arm_motorRunning=false;
//        }
//
//        //For Arm code End==================================================================
//
//        int SLEEPvalue = 600; //500
//        // STEP0:
//        clawOperation = CLAW_CLOSE;
//        clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//        Claw_Servo.setPosition(clawOperation);
//        sleep(100);
//
//        // STEP1:
//        Drive_To_XY_Position(17,5,0.4,0,1);  // Y changed from -15 to -5
//        sleep(SLEEPvalue);
//
//        //STEP2:
//        Turn_Robot(65, 0.5);  // 67
//        sleep(SLEEPvalue);
//
//        //STEP3: Set Arm poistion to 0
//        armTargetPosition = ((int) (ARM_COLLAPSED_INTO_ROBOT));
//        armMotor.setTargetPosition((int) (armTargetPosition));
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(arm_full_speed);
//        while (armMotor.isBusy()){
//            idle();
//        }
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        sleep(SLEEPvalue);
//
//        //STEP4: Position the arm to Hang the specimen
//        armTargetPosition = ((int) (174 * ARM_TICKS_PER_DEGREE)); // 172
//        armMotor.setTargetPosition((int) (armTargetPosition));
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(arm_full_speed);
////                sleep(5000);
//        // new code need to test.
//        while (armMotor.isBusy()){idle();}
//        sleep(SLEEPvalue);
//
//        //STEP5: Pull the back so that specimen can clip to rod
//        Drive_To_XY_Position(17,-3,0.4,0,1);  // Y = -13 to -11
//
//        //STEP6: Open the Claw
//        clawOperation = CLAW_OPEN;
//        clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
//        Claw_Servo.setPosition(clawOperation);
//        sleep(SLEEPvalue);
//
//        //STEP7: Arm go back to 0th position.
//        armTargetPosition = ((int) (ARM_COLLAPSED_INTO_ROBOT));
//        armMotor.setTargetPosition((int) (armTargetPosition));
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(arm_full_speed);
//        while (armMotor.isBusy()){
//            idle();
//        }
//
//        //STEP6 : Reset.
//        imu.resetYaw();
//        sleep(200);
//
//        //STEP7: Rotate 67
//        Turn_Robot(96, 0.75);  //  == 95--96
//        while (FrontRight.isBusy() || FrontLeft.isBusy() || BackRight.isBusy() || BackLeft.isBusy()) { idle();}
//
//        Drive_To_XY_Position(11,10,0.6,0,1);  // Y=7 -- 19
//        while (FrontRight.isBusy() || FrontLeft.isBusy() || BackRight.isBusy() || BackLeft.isBusy()) { idle();}
//
////        while(opModeIsActive()){
////            //Display Global (x, y, theta) coordinates
////            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
////            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
////            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
////
////            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
////            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
////            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());
////
////            telemetry.addData("Thread Active", positionThread.isAlive());
////            telemetry.update();
////        }
//        setPowerAll(0,0,0,0);
//        //Stop the thread
//        globalPositionUpdate.stop();
//    }
//
//    public void Turn_Robot(double TargetedAngle, double Speed){
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//
//        myRobotOrientation = imu.getRobotOrientation(
//                AxesReference.INTRINSIC,
//                AxesOrder.ZYX,
//                AngleUnit.DEGREES
//        );
//        while(YAW_angle < TargetedAngle && opModeIsActive()){
//            FrontRight.setPower(-Speed);
//            BackRight.setPower(-Speed);
//            FrontLeft.setPower(Speed);
//            BackLeft.setPower(Speed);
//
////            robotOrientation = imu.getRobotYawPitchRollAngles();
////            YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
////            telemetry.addData("IMU Angle", YAW_angle);
////            telemetry.update();
////
////            if(YAW_angle < 45){
////                setPowerAll(-Speed, -Speed, Speed, Speed);
////            }else{
////                setPowerAll(-Speed/4, -Speed/4, Speed/4, Speed/4);
////            }
//
//            robotOrientation = imu.getRobotYawPitchRollAngles();
//            YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
//            telemetry.addData("IMU Angle", YAW_angle);
//            telemetry.update();
//        }
//        setPowerAll(0, 0, 0, 0);
//    }
//
//    public void Drive_To_XY_Position(double targetXPosition, double targetYPosition, double robotSpeed, double desiredRobotOrientation, double allowableDistanceError){
//        targetXPosition = targetXPosition * COUNTS_PER_INCH;
//        targetYPosition = targetYPosition * COUNTS_PER_INCH;
//        allowableDistanceError = allowableDistanceError * COUNTS_PER_INCH;
//
//        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
//        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
//        double distanceError = Math.hypot(distanceToXTarget, distanceToYTarget);
//
//        while (opModeIsActive() && (Math.abs(distanceError)) >= allowableDistanceError ){
//            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
//            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
//
//            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
//
//            double robot_movement_X_component = calculateX(robotMovementAngle, robotSpeed);
//            double robot_movement_Y_component = calculateY(robotMovementAngle, robotSpeed);
////            double pivot_Correction = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
//            double pivot_Correction = desiredRobotOrientation;
//
//            // set motor power.
//            double [] motor_speeds = rawSlide(robot_movement_X_component,robot_movement_Y_component,pivot_Correction,robotSpeed);
//            setPowerAll(motor_speeds[3], motor_speeds[2],motor_speeds[1],motor_speeds[0]);
//
//            distanceError = Math.hypot(distanceToXTarget, distanceToYTarget);
//        }
//        telemetry.addLine("Target Achieved : ");
//        telemetry.update();
//        setPowerAll(0,0,0,0);
//    }
//
//    private double calculateX(double desiredAngle, double speed) {
//        return Math.sin(Math.toRadians(desiredAngle)) * speed;
//    }
//
//    private double calculateY(double desiredAngle, double speed) {
//        return Math.cos(Math.toRadians(desiredAngle)) * speed;
//    }
//
//    public double[] rawSlide(double horizontal, double vertical, double pivot, double maxPower){
//        //create an array with all the speeds
//        double[] powers = {vertical-horizontal+pivot, vertical+horizontal+pivot, vertical+horizontal-pivot,vertical-horizontal-pivot};
//        // [BL ,                       FL,                        BR,                       FR   ]
//        //double FRspeed, double BRspeed, double FLspeed, double BLspeed
//        //Only adjust speeds if the robot is moving
//        if(horizontal!=0 || vertical!=0){
//            int max = 0;
//            int counter = 0;
//
//            //determine the maximum speed out of the four motors
//            for(double element:powers){
//                if(Math.abs(element)>Math.abs(powers[max])){
//                    max = counter;
//                }
//                counter++;
//            }
//
//            //set the maximum as a variable
//            double maxCalculatedPower = Math.abs(powers[max]);
//
//            //divide all of the speeds by the max speed to make sure that
//            if(maxCalculatedPower!=0){
//                powers[0]=powers[0]/maxCalculatedPower*maxPower;
//                powers[1]=powers[1]/maxCalculatedPower*maxPower;
//                powers[2]=powers[2]/maxCalculatedPower*maxPower;
//                powers[3]=powers[3]/maxCalculatedPower*maxPower;
//            }
//        }
//        return powers;
//    }
//
//    private void setPowerAll(double FRspeed, double BRspeed, double FLspeed, double BLspeed){
//        FrontRight.setPower(FRspeed);
//        BackRight.setPower(BRspeed);
//        FrontLeft.setPower(FLspeed);
//        BackLeft.setPower(BLspeed);
//    }
//}
