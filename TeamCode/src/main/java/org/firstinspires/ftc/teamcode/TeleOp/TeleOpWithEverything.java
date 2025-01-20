package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOpWithEverything extends LinearOpMode {


    DcMotor fl,fr,bl,br;
    double drive;
    double turn;
    double strafe;
    double drive_speed = 0.8;
    double turn_speed = 0.5;
    double strafe_speed = 0.5;
    double FLspeed, FRspeed, BLspeed, BRspeed;

    DcMotor lift1,lift2;

    Servo arml, armr;

    Servo claw;


    //tuning values with ftc dash

    public static double clawOpen=0;
    public static double clawClose=0;
    public static double armPosHome=0;
    public static double armPosEnterSub=0;
    public static double armPosPickupSub=0;
    public static double armPosBasket=0;
    public static int liftAscent=0;
    public static int liftBasket=0;


    @Override
    public void runOpMode() throws InterruptedException {
        lift1 = hardwareMap.get(DcMotor.class,"lift1");
        lift2 = hardwareMap.get(DcMotor.class,"lift2");

        fl  = hardwareMap.get(DcMotor.class, "left_front");
        bl  = hardwareMap.get(DcMotor.class, "left_back");
        fr = hardwareMap.get(DcMotor.class, "right_front");
        br= hardwareMap.get(DcMotor.class, "right_back");

        arml = hardwareMap.get(Servo.class,"arml");
        armr = hardwareMap.get(Servo.class,"armr");
        claw = hardwareMap.get(Servo.class,"claw");



        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive()) {
            //drive code


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

            fl.setPower(FLspeed);
            fr.setPower(FRspeed);
            bl.setPower(BLspeed);
            br.setPower(BRspeed);

            //drive code end


            //claw code

            if (gamepad2.right_bumper) {
                claw.setPosition(clawOpen);
            }
            if (gamepad2.left_bumper) {
                claw.setPosition(clawClose);
            }

            //claw code end

            //arm code

            if (gamepad2.y) {
                arml.setPosition(armPosBasket);
            }
            if (gamepad2.b) {
                arml.setPosition(armPosEnterSub);
            }
            if (gamepad2.x) {
                arml.setPosition(armPosHome);
            }
            if (gamepad2.a) {
                arml.setPosition(armPosPickupSub);
            }
            //arm code end


            //lift code

            if (gamepad2.dpad_up) {
                lift1.setTargetPosition(liftBasket);
                lift2.setTargetPosition(liftBasket);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.75);
                lift2.setPower(0.75);
            }
            if (gamepad2.dpad_right) {
                lift1.setTargetPosition(liftAscent);
                lift2.setTargetPosition(liftAscent);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.75);
                lift2.setPower(0.75);
            }
            if (gamepad2.dpad_right) {
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.75);
                lift2.setPower(0.75);
            }


            //lift code end
        }
    }
}
