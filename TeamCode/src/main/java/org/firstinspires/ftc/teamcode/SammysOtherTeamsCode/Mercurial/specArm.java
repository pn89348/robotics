package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.Mercurial;

import androidx.annotation.NonNull;

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

public class specArm implements Subsystem {
    // we are working with java, so we don't have the kotlin object class
    // so we will do the work ourselves
    // this instance line is super important
    public static final specArm INSTANCE = new specArm();

    private specArm() {
    }

    // the annotation class we use to attach this subsystem
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

    // Subsystems use the core Feature system of Dairy to be attached to OpModes
    // we need to set up the dependencies, which at its simplest looks like this
    private Dependency<?> dependency =
            // the default dependency ensures that mercurial is attached
            Subsystem.DEFAULT_DEPENDENCY
                    // this is the standard attach annotation that is recommended for features
                    // if you are using other features, you should add them as
                    // dependencies as well
                    // you can also use the annotation to set up and manage
                    // declarative settings for your subsystem, if desired
                    .and(new SingleAnnotation<>(Attach.class));

    // we need to have the getter, rather than the field,
    // but if we actually constructed the dependency every time we ran this, it would slow the program down
    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    private final SubsystemObjectCell<Servo> Elbow = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "specimenelbow"));

    public static Servo getElbow() {
        return INSTANCE.Elbow.get();
    }


    private final SubsystemObjectCell<Servo> Arml = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "specimenarmleft"));

    public static Servo getArml() {
        return INSTANCE.Arml.get();
    }


    private final SubsystemObjectCell<Servo>Armr = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "specimenarmright"));

    public static Servo getarmr() {
        return INSTANCE.Armr.get();
    }

    public void preUserInitHook(@NonNull Wrapper opMode) {
        // default command should be set up here, not in the constructor

        setDefaultCommand(PickupPos());
    }

    // or here
    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {

    }

    // and you might put periodic code in these
    @Override
    public void preUserInitLoopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void preUserLoopHook(@NonNull Wrapper opMode) {
    }

    // or these
    @Override
    public void postUserInitLoopHook(@NonNull Wrapper opMode) {
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }

    // and stopping code can go in here
    @Override
    public void preUserStopHook(@NonNull Wrapper opMode) {
    }

    // or here
    @Override
    public void postUserStopHook(@NonNull Wrapper opMode) {
    }

    // see the feature dev notes on when to use cleanup vs postStop
    @Override
   public void cleanup(@NonNull Wrapper opMode) {
    }

    static Servo SpecArmLeft;
    static Servo SpecArmRight;
    static Servo SpecElbow ;


    public static Lambda HangPos() {
        return new Lambda("HangPos")
                .setInit(() -> {
                    SpecArmLeft = getArml();
                   SpecArmRight = getarmr();
                    SpecElbow = getElbow();

                    SpecElbow.setPosition(0.09);
                    SpecArmLeft.setPosition(0.05);
                    SpecArmRight.setPosition(0.05);
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
    public static Lambda PickupPos() {
        return new Lambda("PickupPos")
                .setInit(() -> {
                    SpecArmLeft = getArml();
                    SpecArmRight = getarmr();
                    SpecElbow = getElbow();

                    SpecArmLeft.setPosition(0.85);
                    SpecArmRight.setPosition(0.85);
                    SpecElbow.setPosition(0.53);

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
