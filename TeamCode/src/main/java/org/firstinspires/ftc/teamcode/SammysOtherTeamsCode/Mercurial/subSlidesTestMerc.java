package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.Mercurial;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.groups.Advancing;

@SubSlides.Attach
@Mercurial.Attach
@TeleOp(name = "SubSlidesMerc")
public class subSlidesTestMerc extends OpMode {
    @Override
    public void init() {
        BoundGamepad Gamepad2 = new BoundGamepad(new SDKGamepad(gamepad2));


        Gamepad2.x().onTrue(new Advancing(SubSlides.Retracted(),SubSlides.Half(),SubSlides.Full()));
    }

    @Override
    public void loop() {

    }
}
