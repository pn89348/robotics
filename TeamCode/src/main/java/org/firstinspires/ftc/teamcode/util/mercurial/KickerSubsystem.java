package org.firstinspires.ftc.teamcode.util.mercurial;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class KickerSubsystem implements Subsystem {
    //ALWAYS CHANGE THIS TO THE RIGHT CLASS
    public static final KickerSubsystem INSTANCE = new KickerSubsystem();
    public static CRServo s;
    private KickerSubsystem() {
    }


    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY
//ALWAYS CHANGE THIS TO THE RIGHT CLASS
                    .and(new SingleAnnotation<>(KickerSubsystem.Attach.class));


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    public void preUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hmap = opMode.getOpMode().hardwareMap;
        s = hmap.get(CRServo.class,"kicker");



    }


    //commands
    public static Lambda kickStart() {
        return new Lambda(" start kick artifact")
                .setInit(() -> {

                    s.setPower(1);

                })
                .setExecute(() -> {

                })
                .setEnd(interrupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(false)
                .setRequirements(INSTANCE)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }
    public static Lambda kickstop() {
        return new Lambda(" stopkick artifact")
                .setInit(() -> {

                    s.setPower(0);

                })
                .setExecute(() -> {

                })
                .setEnd(interrupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(false)
                .setRequirements(INSTANCE)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }


}
