package org.firstinspires.ftc.teamcode.SammysOtherTeamsCode.FTCLIB;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
@Config
public class flywheelPIDtuning extends OpMode {
    //PIDController Feedback;
    PIDFController Feedback;
    SimpleMotorFeedforward feedforward;
    PIDController teest;
    static double kP = 0;
    static double kI =0;
    static double kD = 0;
    static double kS = 0;
    static double kV = 0;
    double f = 0;// placholder cus in the loop it changes the f to the feedforward output
    DcMotorEx flyWheel;
    @Override
    public void init() {

    feedforward = new SimpleMotorFeedforward(kS,kV);
    Feedback = new PIDFController(kP,kI,kD,f);
    flyWheel = hardwareMap.get(DcMotorEx.class,"flywheel");
    }

    @Override
    public void loop() {
        Feedback.setPIDF(kP,kI,kD,f);
        double distance;// gonna use camera for this so ill update this later
        double velocity = 0;// gonna be based on distance or smt
        Feedback.setF(feedforward.calculate(flyWheel.getVelocity(),velocity));
        double power = Feedback.calculate(flyWheel.getVelocity(),velocity);



        flyWheel.setPower(power);

    }
}
