package org.firstinspires.ftc.teamcode.util.test;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mercurial.drivetrainSubsystem;
import org.firstinspires.ftc.teamcode.util.mercurial.intakeRollerSubsystem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.groups.Advancing;


@Mercurial.Attach
@intakeRollerSubsystem.Attach
@drivetrainSubsystem.Attach
@TeleOp(name = "intake + Drive")
public class intakeAndDrive extends OpMode {

    @Override
    public void init() {
        BoundGamepad gamepad2 = Mercurial.gamepad2();
        gamepad2.leftBumper().onTrue(new Advancing(intakeRollerSubsystem.SpinIntake(),intakeRollerSubsystem.StopIntake()));
    }



    @Override
    public void loop() {

    }
}