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

public class SampleClaw implements Subsystem {
    // we are working with java, so we don't have the kotlin object class
    // so we will do the work ourselves
    // this instance line is super important
    public static final SampleClaw INSTANCE = new SampleClaw();

    private SampleClaw() { }

    // the annotation class we use to attach this subsystem
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach{}
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

    private final SubsystemObjectCell<Servo> sampleClaw = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "sampleClaw"));
    public static Servo getSampleClaw() {
        return INSTANCE.sampleClaw.get();
    }

    public void preUserInitHook(@NonNull Wrapper opMode) {
        // default command should be set up here, not in the constructor


        setDefaultCommand(openClaw());
    }
    // or here
    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {

    }
    // and you might put periodic code in these
    @Override
    public void preUserInitLoopHook(@NonNull Wrapper opMode) {}
    @Override
    public void preUserLoopHook(@NonNull Wrapper opMode) {}
    // or these
    @Override
    public void postUserInitLoopHook(@NonNull Wrapper opMode) {}
    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {}

    // and stopping code can go in here
    @Override
    public void preUserStopHook(@NonNull Wrapper opMode) {}
    // or here
    @Override
    public void postUserStopHook(@NonNull Wrapper opMode) {}

    // see the feature dev notes on when to use cleanup vs postStop
    @Override
    public void cleanup(@NonNull Wrapper opMode) {}

   static Servo sampleclaw;
    public static Lambda openClaw(){
        return  new Lambda("openSampleClaw")
                .setInit(()-> {
                    sampleclaw = getSampleClaw();
                    sampleclaw.setPosition(0.15);
                })
                .setExecute(()->{

                })
                .setEnd(interupted-> {

                })
                .setFinish(()->{
                    return true;
                })
                .setInterruptible(false)
                .setRequirements(INSTANCE )
                .setRunStates( Wrapper.OpModeState.ACTIVE);



    }

    public static Lambda closeClaw(){
        return  new Lambda("closeSampleClaw")
                .setInit(()-> {
                    sampleclaw = getSampleClaw();
                    sampleclaw.setPosition(0.4);
                })
                .setExecute(()->{

                })
                .setEnd(interupted-> {

                })
                .setFinish(()->{
                    return true;
                })
                .setInterruptible(false)
                .setRequirements(INSTANCE )
                .setRunStates( Wrapper.OpModeState.ACTIVE);



    }


}
