package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.Mercurial;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
import kotlin.annotation.MustBeDocumented;

public class SubsystemTemplate implements Subsystem {
    public static final SubsystemTemplate INSTANCE = new SubsystemTemplate();

    private SubsystemTemplate() {
    }

    // the annotation class we use to attach this subsystem
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    private Dependency<?> dependency =

            Subsystem.DEFAULT_DEPENDENCY

                    .and(new SingleAnnotation<>(LS.Attach.class));


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    private final SubsystemObjectCell<DcMotorEx> m = subsystemCell(() -> {

        DcMotorEx m = FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "");


        return m;
    });

    public static DcMotorEx getm() {
        return INSTANCE.m.get();
    }

    private final SubsystemObjectCell<Servo> s = subsystemCell(() -> {

        Servo s = FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "");

        return s;
    });

    public static Servo getServo() {
        return INSTANCE.s.get();
    }

    public void preUserInitHook(@NonNull Wrapper opMode) {
        // default command should be set up here, not in the constructor


    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {

    }

    @Override
    public void preUserInitLoopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void preUserLoopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void postUserInitLoopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }


    @Override
    public void preUserStopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void postUserStopHook(@NonNull Wrapper opMode) {
    }


    @Override
    public void cleanup(@NonNull Wrapper opMode) {
    }

    public static Lambda lambda() {
        return new Lambda("do whatever")
                .setInit(() -> {

                })
                .setExecute(() -> {

                })
                .setEnd(interupted -> {

                })
                .setFinish(() -> {
                    return true;
                })
                .setInterruptible(false)
                .setRequirements(INSTANCE)
                .setRunStates(Wrapper.OpModeState.ACTIVE);
    }
}
