package org.firstinspires.ftc.teamcode.util.test;

import android.content.OperationApplicationException;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mercurial.KickerSubsystem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Mercurial.Attach
@KickerSubsystem.Attach
@TeleOp(name = "KickerSubystemTest")
public class kicker extends OpMode {
    @Override
    public void init() {
        BoundGamepad gp2 = Mercurial.gamepad2();
        gp2.leftBumper().onTrue(new Sequential(KickerSubsystem.kickStart(),
                new Wait(1.5),KickerSubsystem.kickstop()));


    }

    @Override
    public void loop() {

    }
}
