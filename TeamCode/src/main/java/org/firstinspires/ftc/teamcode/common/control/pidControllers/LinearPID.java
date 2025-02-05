package org.firstinspires.ftc.teamcode.common.control.pidControllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.ConstantFF;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.DirectionalFF;
import org.firstinspires.ftc.teamcode.common.control.filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.subassems.RobotAssem;

public class LinearPID {

    protected double Kp, Ki, Kd;
    public DirectionalFF feedforward;
    public LowPassFilter filter;
    protected double integralCap;
    protected double a;

    protected double integral = 0;
    protected double prevError = 0;
    protected double prevSetpoint;

    protected ElapsedTime timer = new ElapsedTime();

    public LinearPID(double Kp, double Ki, double Kd, DirectionalFF feedforward, double a, double integralCap){
        setGains(Kp, Ki, Kd);
        this.feedforward = feedforward;
        this.filter = new LowPassFilter(a);
        setIntegralCap(integralCap);
    }

    public LinearPID(double Kp, double Ki, double Kd, double a, double integralCap){
        setGains(Kp, Ki, Kd);
        this.feedforward = new DirectionalFF(0, 0, new Range(0));
        this.filter = new LowPassFilter(a);
        setIntegralCap(integralCap);
    }

    // Set the p, i, and d gains
    public void setGains(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setIntegralCap(double integralCap){
        this.integralCap = integralCap;
    }

    public double getFeedback(double setpoint, double state){
        double error = setpoint - state;
        return getErrorFeedback(error, setpoint);
    }

    protected double getErrorFeedback(double error, double setpoint){
//        Reset stuff if setpoint changes
        if(setpoint != prevSetpoint){
            integral = 0;
            prevError = error;
            filter.reset();
        }
        prevSetpoint = setpoint;
//        Calculate integral
        integral += error * timer.time();
//        Ensure integral does not exceed cap
        if(integral > integralCap) integral = integralCap;
        if(integral < -integralCap) integral = -integralCap;

        double derivative = filter.loop(error - prevError)/timer.time();

        prevError = error;
        timer.reset();

        // Calculate and return feedback
        return (Globals.OPERATING_VOLTAGE / RobotAssem.getVoltage()) * ((Kp * error) + (Ki * integral) + (Kd * derivative) + feedforward.get(error));
    }

    public double getKp(){
        return Kp;
    }

    public double getKi(){
        return Ki;
    }

    public double getKd(){
        return Ki;
    }
}