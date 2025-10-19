package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.test.IndividualTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.FTCLIB.ClawPivot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name = "ClawPivotSubsystemTest")
public class ClawPivotSubSystemTest extends LinearOpMode {

    boolean GoingClockWise;

    ClawPivot clawPivot;



    @Override
    public void runOpMode() throws InterruptedException {

        GoingClockWise = true;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        clawPivot = new ClawPivot(hardwareMap, "wrist");

        waitForStart();

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad2.y && !previousGamepad2.y) {
            if (GoingClockWise) {
                clawPivot.PivotGoingClockWise();
            } else {
                clawPivot.PivotGoingCounterClockwise();
            }
        }
        if (clawPivot.GetClawPivot() == 0.65 || clawPivot.GetClawPivot() == 0) {
            GoingClockWise = !GoingClockWise;
        }
    }
}

