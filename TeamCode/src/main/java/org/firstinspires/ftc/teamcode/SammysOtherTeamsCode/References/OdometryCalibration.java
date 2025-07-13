package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.References;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@Autonomous(name = "OdometryCalibration-GJ", group = "Calibration")
//@Disabled
public class OdometryCalibration extends LinearOpMode {
    //Drive motors
    DcMotor FrontRight, BackRight, FrontLeft, BackLeft;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    //IMU Sensor
    private final IMU             imu         = null;      // Control/Expansion Hub IMU
    Orientation myRobotOrientation;
    YawPitchRollAngles robotOrientation;
    double YAW_angle;
    //NEW12_2024
    double Pitch_angle;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String frName = "fr", brName = "br", flName = "fl", blName = "bl";
    String verticalLeftEncoderName = frName, verticalRightEncoderName = flName, horizontalEncoderName = blName;

    final double PIVOT_SPEED = 0.25;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
//    final double COUNTS_PER_INCH = 307.699557;
    static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;   //537.7 eg: GoBILDA 312 RPM Yellow Jacket
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     //1.0 No External Gearing.
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
    static final double     WHEEL_DIAMETER_INCHES   = 5.51181 ;     //4.0 For figuring circumference
    // 140mm wheels: Convert mm to inchs (140mm is 5.51181 inches)
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.setMsTransmissionInterval(20);

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardwareMap(frName, brName, flName, blName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        myRobotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );

        // Create an object to receive the IMU angles
//        YawPitchRollAngles robotOrientation;
//        robotOrientation = imu.getRobotYawPitchRollAngles();

        imu.resetYaw();

        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();
        sleep(2000);

        waitForStart();

//        while(opModeIsActive()){
//            robotOrientation = imu.getRobotYawPitchRollAngles();
//            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
//            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
//            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
//            telemetry.addData("Yaw=", Yaw);
//            telemetry.addData("Pitch=", Pitch);
//            telemetry.addData("Roll=", Roll);
//            telemetry.update();
//            FrontRight.setPower(-PIVOT_SPEED);
//            BackRight.setPower(-PIVOT_SPEED);
//            FrontLeft.setPower(PIVOT_SPEED);
//            BackLeft.setPower(PIVOT_SPEED);
//        }

        robotOrientation = imu.getRobotYawPitchRollAngles();
        YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("IMU YAW Angle", YAW_angle);
        //NEW12_2024
        Pitch_angle = robotOrientation.getPitch(AngleUnit.DEGREES);
        telemetry.addData("IMU PITCH Angle", Pitch_angle);
        telemetry.update();


        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
//        while(getZAngle() < 90 && opModeIsActive()){
        while(YAW_angle < 90 && opModeIsActive()){
//            telemetry.addData("angle=", getZAngle());
//            telemetry.update();
//            telemetry.addData("firstAngle=", myRobotOrientation.firstAngle);
//            telemetry.addData("secondAngle=", myRobotOrientation.secondAngle);
//            telemetry.addData("thirdAngle=", myRobotOrientation.thirdAngle);
//            telemetry.update();
//            sleep(2000);
            FrontRight.setPower(-PIVOT_SPEED);
            BackRight.setPower(-PIVOT_SPEED);
            FrontLeft.setPower(PIVOT_SPEED);
            BackLeft.setPower(PIVOT_SPEED);

            robotOrientation = imu.getRobotYawPitchRollAngles();
            YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU Angle", YAW_angle);
            telemetry.update();

            if(YAW_angle < 45){
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            }else{
                setPowerAll(-PIVOT_SPEED/4, -PIVOT_SPEED/4, PIVOT_SPEED/4, PIVOT_SPEED/4);
            }

            robotOrientation = imu.getRobotYawPitchRollAngles();
            YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU Angle", YAW_angle);
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            robotOrientation = imu.getRobotYawPitchRollAngles();
            YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU Angle", YAW_angle);
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        robotOrientation = imu.getRobotYawPitchRollAngles();
        YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
        double angle = YAW_angle;

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(verticalLeft.getCurrentPosition()) + (Math.abs(verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        robotOrientation = imu.getRobotYawPitchRollAngles();
        YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
//        horizontalTickOffset = horizontal.getCurrentPosition()/Math.toRadians(getZAngle());
        horizontalTickOffset = horizontal.getCurrentPosition()/Math.toRadians(YAW_angle);

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            robotOrientation = imu.getRobotYawPitchRollAngles();
            YAW_angle   = -robotOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU Angle", YAW_angle);
            telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

    private void initHardwareMap(String frName, String brName, String flName, String blName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        FrontRight = hardwareMap.dcMotor.get(frName);
        BackRight = hardwareMap.dcMotor.get(brName);
        FrontLeft = hardwareMap.dcMotor.get(flName);
        BackLeft = hardwareMap.dcMotor.get(blName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
        sleep(3000);

    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
//    private double getZAngle(){
//       return
//               (-imu.getAngularOrientation().firstAngle);
//    }
//    private double getZAngle(){
////        // Then read or display the desired values (Java type float):
//        robotOrientation = imu.getRobotYawPitchRollAngles();
//        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
//        return (-Yaw);
//    }
//    private double getZAngle(){
//       return
//               (-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//    }



    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        FrontRight.setPower(rf);
        BackRight.setPower(rb);
        FrontLeft.setPower(lf);
        BackLeft.setPower(lb);
    }

}
