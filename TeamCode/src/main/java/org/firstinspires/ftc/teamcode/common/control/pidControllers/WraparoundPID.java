package org.firstinspires.ftc.teamcode.common.control.pidControllers;

import org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.DirectionalFF;

public class WraparoundPID extends LinearPID {

    public WraparoundPID(double Kp, double Ki, double Kd, DirectionalFF feedforward, double a, double integralCap){
        super(Kp, Ki, Kd, feedforward, a, integralCap);
    }

    public WraparoundPID(double Kp, double Ki, double Kd, double a, double integralCap){
        super(Kp, Ki, Kd, a, integralCap);
    }

    // Get Feedback
    public double getFeedback(double setpoint, double state){
        // Calculate Error
        double error = UsefulStuff.normalizeAngle(setpoint - state);
        // Calculate and return feedback
        return getErrorFeedback(error, setpoint);
    }
}