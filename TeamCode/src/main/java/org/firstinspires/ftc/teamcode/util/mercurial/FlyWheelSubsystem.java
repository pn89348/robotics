package org.firstinspires.ftc.teamcode.util.mercurial;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class FlyWheelSubsystem implements Subsystem {
    //ALWAYS CHANGE THIS TO THE RIGHT CLASS
    public static final FlyWheelSubsystem INSTANCE = new FlyWheelSubsystem();
    public static DcMotorEx m;
    private FlyWheelSubsystem() {
    }


    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY
//ALWAYS CHANGE THIS TO THE RIGHT CLASS
                    .and(new SingleAnnotation<>(FlyWheelSubsystem.Attach.class));


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }


    //init hook
    public void preUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hmap = opMode.getOpMode().hardwareMap;
        m = hmap.get(DcMotorEx.class,"flywheel");
        // other stuff for init of motors/ servos go here


    }



    //commands

    public static Lambda Shoot() {
        return new Lambda("Shoot Artifact")
                .setInit(() -> {

                    m.setPower(1);
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
    public static Lambda stop() {
        return new Lambda("stop flywheel")
                .setInit(() -> {

                    m.setPower(0);
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
