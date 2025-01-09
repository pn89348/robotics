package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton : RR Strategy Test", group="Robot")
public class Auton2 extends LinearOpMode {
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
                        .strafeToLinearHeading(new Vector2d(20, 20), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .strafeToConstantHeading(new Vector2d(20, 0))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(0, 0))
                        .waitSeconds(0.5)
                        .turnTo(Math.toRadians(0))
                        .waitSeconds(0.5)
                        .build());
    }
}