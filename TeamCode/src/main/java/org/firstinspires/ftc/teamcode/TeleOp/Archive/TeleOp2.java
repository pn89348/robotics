package org.firstinspires.ftc.teamcode.TeleOp.Archive;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name="TeleOp 2", group="In Progress TeleOp")
public class TeleOp2 extends LinearOpMode {

	// Declare OpMode members for each of the 4 motors.
	private ElapsedTime runtime = new ElapsedTime();
	private DcMotor leftFrontDrive = null;
	private DcMotor leftBackDrive = null;
	private DcMotor rightFrontDrive = null;
	private DcMotor rightBackDrive = null;
	private DcMotor liftL = null;
	private DcMotor liftR = null;
	private Servo armL = null;
	private Servo armR = null;
	private Servo claw = null;
	GoBildaPinpointDriver odo;

	// Values for different motor positions
	public static int maxLiftHeight = 3190;
	public static int liftAscent = maxLiftHeight;
	public static int liftBasket = maxLiftHeight;
	public static double clawOpen = 0.2833; // slightly wider than horizontal sample
	public static double clawClose = 0.5; // enough to hold sample in smallest orientation
	public static double armStarting = 1; // or 0.98ish

	public static double armSample = 0.1028; // height to pick up sample when lift is at bottom
	public static double armSubLow = 0.1461;
	public static double armSubHigh = 0.3067;
	public static double armPosBasket = 0.4533;
	public static double armPosAscent = 0.545;

	public static int liftPullDown=2150;
	public static double armPullDown = 0.25;

	

	double oldTime = 0;

	double maxDrivePower = 1;
	double maxLiftPower = 1;
	double powerChangeSensitivity = 0.25;

	@Override
	public void runOpMode() {

		// Initialize the hardware variables. Note that the strings used here must correspond
		// to the names assigned during the robot configuration step on the DS or RC devices.
		leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
		leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
		rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

		odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

		odo.setOffsets(-84.0, 168.0); // measure and come back to this
		odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		odo.resetPosAndIMU();

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
		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
		rightBackDrive.setDirection(DcMotor.Direction.REVERSE);



		// Wait for the game to start (driver presses PLAY)
		telemetry.addData("X offset", odo.getXOffset());
		telemetry.addData("Y offset", odo.getYOffset());
		telemetry.addData("Device Version Number:", odo.getDeviceVersion());
		telemetry.addData("Device Scalar", odo.getYawScalar());
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();
		runtime.reset();

		boolean prevRightBumper = false;
		boolean prevLeftBumper = false;

		// run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {
			odo.update();
			if (gamepad1.a){
				odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
			}
			if (gamepad1.b){
				odo.recalibrateIMU(); //recalibrates the IMU without resetting position
			}
			double newTime = getRuntime();
			double loopTime = newTime-oldTime;
			double frequency = 1/loopTime;
			oldTime = newTime;
			Pose2D pos = odo.getPosition();
			String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
			telemetry.addData("Position", data);

			Pose2D vel = odo.getVelocity();
			String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
			double[] acc = {vel.getX(DistanceUnit.MM) / loopTime, vel.getY(DistanceUnit.MM) / loopTime, vel.getHeading(AngleUnit.DEGREES) / loopTime};
			String accelerationDisplay = String.format(Locale.US,"{XAcc: %.3f, YAcc: %.3f, HAcc: %.3f}", acc[0], acc[1], acc[1]);

			double max;

			// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
			double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
			double lateral =  gamepad1.left_stick_x;
			double yaw	 =  gamepad1.right_stick_x;

			// Combine the joystick requests for each axis-motion to determine each wheel's power.
			// Set up a variable for each drive wheel to save the power level for telemetry.
			double leftFrontPower  = axial + lateral + yaw;
			double rightFrontPower = axial - lateral - yaw;
			double leftBackPower   = axial - lateral + yaw;
			double rightBackPower  = axial + lateral - yaw;

			// Normalize the values so no wheel power exceeds 100%
			// This ensures that the robot maintains the desired motion.
			max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
			max = Math.max(max, Math.abs(leftBackPower));
			max = Math.max(max, Math.abs(rightBackPower));

			if (max > 1) {
				leftFrontPower  /= max;
				rightFrontPower /= max;
				leftBackPower   /= max;
				rightBackPower  /= max;
			}

			boolean rightBumper = gamepad1.right_bumper;
			boolean leftBumper = gamepad1.left_bumper;
			if (prevRightBumper && !rightBumper) { // on release of button
				maxDrivePower += powerChangeSensitivity;
			}
			if (prevLeftBumper && !leftBumper) {
				maxDrivePower -= powerChangeSensitivity;
			}
			prevRightBumper = rightBumper;
			prevLeftBumper = leftBumper;
			if (maxDrivePower < 0) {
				maxDrivePower = 0;
			} else if (maxDrivePower > 1) {
				maxDrivePower = 1;
			}


			leftFrontPower  *= maxDrivePower;
			rightFrontPower *= maxDrivePower;
			leftBackPower   *= maxDrivePower;
			rightBackPower  *= maxDrivePower;

			/*
			leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
			leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
			rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
			rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
			*/

			if (gamepad1.x) { // Stop all motors button
				leftFrontPower  = 0;
				rightFrontPower = 0;
				leftBackPower   = 0;
				rightBackPower  = 0;
			}

			// Send calculated power to wheels
			leftFrontDrive.setPower(leftFrontPower);
			rightFrontDrive.setPower(rightFrontPower);
			leftBackDrive.setPower(leftBackPower);
			rightBackDrive.setPower(rightBackPower);

			// Show the elapsed game time and wheel power.

			telemetry.addData("Velocity", velocity);
			telemetry.addData("Status", odo.getDeviceStatus());
			telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
			telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

			telemetry.addData("Status", "Run Time: " + runtime.toString());
			telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
			telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
			telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
			telemetry.addData("Left Stick X", gamepad1.left_stick_x);
			telemetry.addData("Right Stick", gamepad1.right_stick_x);
			telemetry.addData("Power", maxDrivePower);
			telemetry.update();
		}

		leftFrontDrive.setPower(0);
		leftBackDrive.setPower(0);
		rightFrontDrive.setPower(0);
		rightBackDrive.setPower(0);
	}
}
