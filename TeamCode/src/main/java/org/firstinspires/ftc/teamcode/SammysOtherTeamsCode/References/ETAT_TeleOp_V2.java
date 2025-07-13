package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;//import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//@Disabled
@TeleOp (name = "ETAT_TeleOp_V2", group = "ETAT")
public class ETAT_TeleOp_V2 extends LinearOpMode {
    // get an instance of the "ETAT_functions" class.
    private final ETAT_functions_v1 ETAT_Robot_functions = new ETAT_functions_v1(this);

    //Hardware Map Names for drive motors and odometry wheels.
    String frName = "fr", brName = "br", flName = "fl", blName = "bl";
    String FrontLeftEncoderName = frName, FrontRightEncoderName = flName, BackMiddleEncoderName = blName;
    String IMUname = "imu";
    String LeftLinearSlideName = "lsl", RightLinearSlideName = "lsr";
    String ClawServoName = "claw";
    String ArmName = "arm";
    String ActiveIntakeCRServoName = "activeintake";

    // For Drive Train

    // For Linear Slide

    // For Claw
    public final static double CLAW_HOME = 0.0;
    public final static double CLAW_MIN_RANGE = 0.0;
    public final static double CLAW_MAX_RANGE = 0.5;
    public final static double CLAW_CLOSE = 0.0;
    public final static double CLAW_OPEN = 0.25;
    double clawOperation = CLAW_HOME;

    // For Arm

    // For Active-Intake

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
                LeftLinearSlideName, RightLinearSlideName,
                ClawServoName,
                ActiveIntakeCRServoName,
                ArmName
        );

        // Robot Hardware Configuration
        ETAT_Robot_functions.myRobot_HardwareConfig();

        ETAT_Robot_functions.TelemetryShowHeading("Initialization & Configuration : DONE");
        ETAT_Robot_functions.TelemetryShowHeading("> Wait for Start ......");
        ETAT_Robot_functions.TelemetryUpdate(10);

        waitForStart();

        while (opModeIsActive()) {
            //Drive Train code Start===============================================================
            ETAT_Robot_functions.Drivetrain(gamepad1.left_stick_y , gamepad1.right_stick_x, gamepad1.left_stick_x,
                    0.4, 0.4, 0.4);
            // Drive Train code end

            // For Linear Slide Code start==========================================================
            if (gamepad2.dpad_up) {
                ETAT_Robot_functions.set_Left_Right_LinearSlide_Height_Sync_Control_v1("UP", 2800, 0.7); // working
            }
            if (gamepad2.dpad_down) {
                ETAT_Robot_functions.set_Left_Right_LinearSlide_Height_Sync_Control_v1("DOWN", 5, 0.7);  // working
            }
            // For Linear Slide Code end

            //For Claw code start==================================================================
            boolean rightBumper = gamepad2.right_bumper;
            boolean leftBumper = gamepad2.left_bumper;
            if (rightBumper) {
                clawOperation = CLAW_CLOSE;
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                ETAT_Robot_functions.Claw_Servo.setPosition(clawOperation);
            }
            if (leftBumper) {
                clawOperation = CLAW_OPEN;
                clawOperation = Range.clip(clawOperation, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
                ETAT_Robot_functions.Claw_Servo.setPosition(clawOperation);
            }
            // For Claw code end

            //For Arm code start==================================================================
            // Home
            if (gamepad2.b) {   // Home
                ETAT_Robot_functions.setArm_version2("HOME", 5, 250);
            }
            // UP
            if (gamepad2.y) {
                ETAT_Robot_functions.setArm_version2("DROP_or_CLIP", -655, 250);
            }
            // DOWN
            if (gamepad2.a) {
                ETAT_Robot_functions.setArm_version2("PICKUP", -990, 250);
            }

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