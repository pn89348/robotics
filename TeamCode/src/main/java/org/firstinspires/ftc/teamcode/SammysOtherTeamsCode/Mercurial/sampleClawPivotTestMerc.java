package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.Mercurial;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Mercurial.Attach
@sampleClawPivot.Attach
@TeleOp(name = "ClawPivotMerc")
public class sampleClawPivotTestMerc extends OpMode {
    @Override
    public void init() {
        BoundGamepad Gamepad2 = new BoundGamepad(new SDKGamepad(gamepad2));


        Gamepad2.y().onTrue(sampleClawPivot.PivotStuff());
    }

    @Override
    public void loop() {

    }
}
