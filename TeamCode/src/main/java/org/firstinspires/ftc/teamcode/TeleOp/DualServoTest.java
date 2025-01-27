package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Dual Servo Test", group="Component Testing")
public class DualServoTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
//
//    GoBildaPinpointDriver odo;

    private Servo servo1 = null;
    private Servo servo2 = null;
    double oldTime = 0;

//    double maxDrivePower = 0.5; // Set to 1 for full power
//    double powerChangeSensitivity = 0.25;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
//
//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
//
//        odo.setOffsets(-84.0, 168.0); // measure and come back to this
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        // DataLogger datalogger = new DataLogger("myDatalog.txt");


        // ########################################################################################
        // !!!			IMPORTANT Drive Information. Test your motor directions.			!!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        boolean prevRightBumper = false;
        boolean prevLeftBumper = false;

        double currentPos1 = servo1.getPosition();
        double currentPos2 = servo2.getPosition();
//        // Wait for the game to start (driver presses PLAY)
//        telemetry.addData("X offset", odo.getXOffset());
//        telemetry.addData("Y offset", odo.getYOffset());
//        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
//        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.addData("Servo 1 Pos:", servo1.getPosition());
        telemetry.addData("Servo 2 Pos:", servo2.getPosition());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // make the servo position increase when right trigger is pressed and decrease when left trigger is pressed, based on how far the trigger is pressed
            double trigger = gamepad1.right_trigger - gamepad1.left_trigger;
            double newPos = currentPos1 + trigger/200;
            newPos = Math.min(1.0, Math.max(0.0, newPos));
            servo1.setPosition(newPos);
            servo2.setPosition(newPos);

            if (gamepad1.y) {
                servo1.setPosition(0.5);
                servo2.setPosition(0.5);
            } else if (gamepad1.b) {
                servo1.setPosition(0.0);
                servo2.setPosition(0.0);
            } else if (gamepad1.x) {
                servo1.setPosition(1.0);
                servo2.setPosition(1.0);
            }
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
            double oldPos1 = currentPos1;
            currentPos1 = servo1.getPosition();
            double velocity = (currentPos1 - oldPos1) / loopTime;
            double posDiff = currentPos2 - currentPos1;
            telemetry.addData("Servo 1 Pos:", currentPos1);
            telemetry.addData("Servo 1 Vel:", velocity);
            telemetry.addData("Servo 2 Pos:", currentPos2);
            telemetry.addData("Servo 2 Pos Diff:", posDiff);
            telemetry.addData("Triggers:", trigger);
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
