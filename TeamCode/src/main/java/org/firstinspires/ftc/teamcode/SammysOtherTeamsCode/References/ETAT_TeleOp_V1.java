package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;

//import android.util.Range;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp (name = "ETAT_TeleOp_V1", group = "ETAT")
public class ETAT_TeleOp_V1 extends LinearOpMode {
    // get an instance of the "ETAT_functions" class.
    private final ETAT_functions_v1 ETAT_Robot_functions = new ETAT_functions_v1(this);

    //Hardware Map Names for drive motors and odometry wheels.
    String frName = "fr", brName = "br", flName = "fl", blName = "bl";
    String FrontLeftEncoderName = frName, FrontRightEncoderName = flName, BackMiddleEncoderName = blName;
    String IMUname = "imu";
    //    String  LinearSlideName = "fl";  // "ls"
    String LeftLinearSlideName = "fl", RightLinearSlideName = "fr";
    //    String LeftLinearSlideName = "LLS", RightLinearSlideName = "RLS";
    String ClawServoName = "claw";
    String ActiveIntakeCRServoName = "activeintake";
    String ArmName = "bl";   // "arm"

    // For Drive Train


    // For Linear Slide
    double SLIDE_UP_POWER = 0.5;
    double SLIDE_DOWN_POWER = -0.5;

    // For Claw
    public final static double CLAW_HOME = 0.0;
    public final static double CLAW_MIN_RANGE = 0.0;
    public final static double CLAW_MAX_RANGE = 0.5;
    public final static double CLAW_CLOSE = 0.0;
    public final static double CLAW_OPEN = 0.2;
    double clawOperation = CLAW_HOME;

    // For Active-Intake

    // For Arm
//    public DcMotor  armMotor    = null; //the arm motor
//    final double ARM_TICKS_PER_DEGREE =
//            28 // number of encoder ticks per rotation of the bare motor
//                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
//                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
//                    * 1/360.0; // we want ticks per degree, not per rotation
//
//    final double ARM_COLLAPSED_INTO_ROBOT  = 1 * ARM_TICKS_PER_DEGREE;;
//    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Telemetry.
        ETAT_Robot_functions.TelemetryInit(250); // Default refresh rate is 250ms
        ETAT_Robot_functions.TelemetryUpdate(10);

        //Robot Hardware Mapping.
        ETAT_Robot_functions.myRobot_HardwareMap(
                frName, brName, flName, blName,
                FrontLeftEncoderName, FrontRightEncoderName, BackMiddleEncoderName,
                IMUname,
//                LinearSlideName,
                LeftLinearSlideName, RightLinearSlideName,
                ClawServoName,
                ActiveIntakeCRServoName,
                ArmName
        );

        // Robot Hardware Configuration
        ETAT_Robot_functions.myRobot_HardwareConfig();

//        armMotor   = hardwareMap.get(DcMotor.class, "bl"); //the arm motor
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
//        armMotor.setTargetPosition(0);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ETAT_Robot_functions.TelemetryShowHeading("Initialization & Configuration : DONE");
        ETAT_Robot_functions.TelemetryShowHeading("> Wait for Start ......");
        ETAT_Robot_functions.TelemetryUpdate(10);

        waitForStart();

        while (opModeIsActive()){

//            //Drive Train code Start===============================================================
//            ETAT_Robot_functions.Drivetrain(gamepad1.left_stick_y , gamepad1.right_stick_x, gamepad1.left_stick_x,
//                                            0.4, 0.4, 0.4);
//            // Drive Train code end

            // For Linear Slide Code start==========================================================
            if(gamepad1.dpad_up) {
//                ETAT_Robot_functions.setLinearSlide("UP", 3500, 0.3);
                ETAT_Robot_functions.set_Left_Right_LinearSlide_v2("UP", 2600, 0.7);
            }
            if(gamepad1.dpad_down) {
//                ETAT_Robot_functions.setLinearSlide("DOWN", 0, 0.3);
                ETAT_Robot_functions.set_Left_Right_LinearSlide_v2("DOWN", 3, 0.7);
            }
            // For Linear Slide Code end

            //For Claw code start==================================================================
            if (gamepad1.a) {
                clawOperation = CLAW_CLOSE;
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                ETAT_Robot_functions.Claw_Servo.setPosition(clawOperation);
//                sleep(100);
            }
            else if (gamepad1.y) {
                clawOperation = CLAW_OPEN;
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                ETAT_Robot_functions.Claw_Servo.setPosition(clawOperation);
//                sleep(100);
            }
//            else
//                ETAT_Robot_functions.Claw_Servo.close();
            // For Claw code end


            //For Arm code start==================================================================
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;

            if(gamepad1.b) {   // Home
                ETAT_Robot_functions.setArm_version2("HOME", 5, 250);
            }
//            else if (gamepad1.dpad_left){  // Up
            else if (rightBumper){
                ETAT_Robot_functions.setArm_version2("DROP_or_CLIP", 655, 250);
            }
//            else if (gamepad1.dpad_right){   // Down
            else if (leftBumper){
                ETAT_Robot_functions.setArm_version2("PICKUP", 950, 250); //990
            }

//            ETAT_Robot_functions.setArmnew(gamepad1.x, gamepad1.right_bumper, gamepad1.left_bumper, ArmPosition, Speed);

//            if (gamepad1.dpad_left){
//                armPosition = 655;
//                armMotor.setTargetPosition((int) (armPosition));
//                ((DcMotorEx) armMotor).setVelocity(125);  //2100
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else if (gamepad1.dpad_right){
//                armPosition = 850;
//                armMotor.setTargetPosition((int) (armPosition));
//                ((DcMotorEx) armMotor).setVelocity(150);  //2100
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while (armMotor.isBusy()){
//                    idle();
//                }
//                armPosition = 990;
//                armMotor.setTargetPosition((int) (armPosition));
//                ((DcMotorEx) armMotor).setVelocity(15);  //2100
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                while (armMotor.isBusy()){
////                    idle();
////                }
//            }
//
//            if (gamepad1.b){
//                armPosition = 5;
//                armMotor.setTargetPosition((int) (armPosition));
//                ((DcMotorEx) armMotor).setVelocity(100);  //2100
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            if (((DcMotorEx) armMotor).isOverCurrent()){
//                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
//            }
//
//            /* send telemetry to the driver of the arm's current position and target position */
//            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
//            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
//            telemetry.update();

            //For Arm code End==================================================================

//            //For Active-Intake code start==================================================================
//            while(gamepad1.b)
//                ETAT_Robot_functions.setActiveIntake("IN");
//            while(gamepad1.x)
//                ETAT_Robot_functions.setActiveIntake("OUT");
//            ETAT_Robot_functions.setActiveIntake("STOP");
//            //For Active-Intake code End==================================================================

        }
    }
}
