package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.Mercurial;



import androidx.annotation.NonNull;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.util.OpModeLazyCell;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;
import kotlin.annotation.MustBeDocumented;

public class LS implements Subsystem {
    // we are working with java, so we don't have the kotlin object class
    // so we will do the work ourselves
    // this instance line is super important
    public static final LS INSTANCE = new LS();

    private LS() { }

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
//     SubsystemObjectCells get eagerly reevaluated at the start of every OpMode, if this subsystem is attached
//     this means that we can always rely on motor to be correct and up-to-date for the current OpMode
//     this can also work with Calcified
    private final SubsystemObjectCell<DcMotorEx> lsl = subsystemCell(() ->{

      DcMotorEx m =  FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "lsl");

      return m;
    });
    public static DcMotorEx getlsl() {
        return INSTANCE.lsl.get();
    }
    private final SubsystemObjectCell<DcMotorEx> lsr =  subsystemCell(()-> {
        DcMotorEx m = FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "lsr");
        m.setDirection(DcMotorSimple.Direction.REVERSE);
        return m;
    });
    public static DcMotorEx getlsr() {
        return INSTANCE.lsr.get();
    }









    static int target;
    private static PIDController controller;
    public static double p =0.02,i=0, d =0;
    public static double f=0.15;

    // we get the full benefit of the Dairy core feature set,
    // so we can use any hooks to run code around the code we end up writing
    // this gives us a lot of freedom to set up a complex and powerful subsystem

    // init code might go in here

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        // default command should be set up here, not in the constructor
        setDefaultCommand(setTargetPos0());
        controller = new PIDController(p, i, d);


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

    // all depending on what you need!
    // remember, you only need to write implementations for the hooks you actually use
    // the rest don't need to be added to the class, nice and clean

    //
    // Commands
    //
    // commands are the same as older mercurial!
    // lambda commands are once again, powerful tools for developing simple units of operation
    static DcMotorEx lsr1;
    static DcMotorEx lsl1;

    @NonNull
    public static Lambda setTargetPos100(){
        return  new Lambda("setTargetPos100")
                .setInit(()->{
                    target = 100;
                    lsl1 = getlsr();
                    lsr1 = getlsr();
                })
                .setExecute(()->{
                    controller.setPID(p, i, d);
                    //  double BothLSCurrentPos = lsl.getCurrentPosition()*0.5 + lsr.getCurrentPosition()*0.5;
//            double lsAveragePos = BothLSCurrentPos;

                    double lsAveragePos = lsl1.getCurrentPosition();
          /* the method for this is that the position for lsl and lsr is generally the the same. I have included a
          commented out code which gets both, I just didnt want it to be too complex
           */
                    double pid = controller.calculate(lsAveragePos,target);
                    double ff = f;
                    double power = pid + ff;

                    lsl1.setPower(power);
                    lsr1.setPower(power);
                    })
                .setEnd(interupted -> {

                })
                .setFinish(()-> {
                    return true;
                })
                .setInterruptible(true)
                .setRequirements(Mercurial.INSTANCE )
                .setRunStates(Wrapper.OpModeState.INIT, Wrapper.OpModeState.ACTIVE);


    }
    public static Lambda setTargetPos0(){
        return  new Lambda("setTargetPos0")
                .setInit(()-> {
                    target = 0;
                    lsl1 = getlsr();
                    lsr1 = getlsr();
                })
                .setExecute(()->{
                    controller.setPID(p, i, d);
                    //  double BothLSCurrentPos = lsl.getCurrentPosition()*0.5 + lsr.getCurrentPosition()*0.5;
//            double lsAveragePos = BothLSCurrentPos;

                    double lsAveragePos = lsl1.getCurrentPosition();
          /* the method for this is that the position for lsl and lsr is generally the the same. I have included a
          commented out code which gets both, I just didnt want it to be too complex
           */
                    double pid = controller.calculate(lsAveragePos,target);
                    double ff = f;
                    double power = pid + ff;

                    lsl1.setPower(power);
                    lsr1.setPower(power);})
                .setEnd(interupted -> {

                })
                .setFinish(()-> {
                    return true;
                })
                .setInterruptible(true)
                .setRequirements(INSTANCE )
                .setRunStates( Wrapper.OpModeState.ACTIVE);


    }


    public static Lambda setTargetPos(int targetPos){
        return  new Lambda("setTargetPos0")
                .setInit(()-> {
                    target = targetPos;
                    lsl1 = getlsr();
                    lsr1 = getlsr();
                })
                .setExecute(()->{
                    controller.setPID(p, i, d);
                    //  double BothLSCurrentPos = lsl.getCurrentPosition()*0.5 + lsr.getCurrentPosition()*0.5;
//            double lsAveragePos = BothLSCurrentPos;

                    double lsAveragePos = lsl1.getCurrentPosition();
          /* the method for this is that the position for lsl and lsr is generally the the same. I have included a
          commented out code which gets both, I just didnt want it to be too complex
           */
                    double pid = controller.calculate(lsAveragePos,target);
                    double ff = f;
                    double power = pid + ff;

                    lsl1.setPower(power);
                    lsr1.setPower(power);})
                .setEnd(interupted -> {

                })
                .setFinish(()-> {
                    return true;
                })
                .setInterruptible(true)
                .setRequirements(INSTANCE )
                .setRunStates( Wrapper.OpModeState.ACTIVE);


    }





}


