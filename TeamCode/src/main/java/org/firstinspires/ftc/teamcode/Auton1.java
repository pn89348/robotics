package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import com.acmerobotics.roadrunner.ftc.PinpointEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name="Auton 1: RR Test", group="Robot")
public class Auton1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Pose2d currentPose = drive.pose;

        waitForStart();

        if(isStopRequested()) return;

        /*Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(18, 12), Math.toRadians(45))
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(18, 12, 0))
                        .strafeToLinearHeading(new Vector2d(18, 12), Math.toRadians(180))
                        .build());*/
        /*Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToX(10)
                        .waitSeconds(0.5)
                        .lineToX(0)
                        .waitSeconds(0.5)
                        .turnTo(Math.toRadians(90))
                        .waitSeconds(1)
                        .lineToY(10)
                        .waitSeconds(0.5)
                        .lineToY(0)
                        .waitSeconds(0.5)
                        .turnTo(Math.toRadians(0))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(5, 10))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(0, 0))
                        .build());*/
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .turnTo(Math.toRadians(90))
                        .waitSeconds(1)
                        .turnTo(Math.toRadians(180))
                        .turnTo(Math.toRadians(0))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(10, 10), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .strafeToConstantHeading(new Vector2d(10, 0))
                        .waitSeconds(0.5)
                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0))
                        .waitSeconds(0.5)
                        .build());
    }
}