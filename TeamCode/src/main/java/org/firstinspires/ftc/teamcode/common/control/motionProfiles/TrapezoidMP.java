package org.firstinspires.ftc.teamcode.common.control.motionProfiles;

import static java.lang.Math.pow;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.control.geometry.Constraints;

public class TrapezoidMP {

    public final Constraints extend, retract;
    public Constraints active;
    private double t1, t2, t3; // End times
    private double c1, c2, c3, c4; // End positions (the + a constant when you integrate the trapezoidal velocity)
    private double maxVelo;
    private final ElapsedTime timer;

    public TrapezoidMP(Constraints extend, Constraints retract){
        this.extend = extend;
        this.retract = retract.reverse();
        timer = new ElapsedTime();
    }

    public TrapezoidMP(Constraints constraints){
        this(constraints, constraints);
    }

    /**
     * Call in the setPosition() method of a subassembly, generates the motion profile to get to that position
     * @param initState the current state of the subassembly
     * @param finalSetpoint the new setpoint of the subassembly
     */
    public void setProfile(double initState, double finalSetpoint){
        double distance = finalSetpoint - initState;
        int direction = (int)signum(distance);
        switch(direction){
            case 1: active = extend; break;
            case -1: active = retract; break;
            default: return;
        }
        double l1 = active.velo / active.accel;
        double l3 = -active.velo / active.decel;
        double l2 = distance / active.velo - (l1 + l3) / 2;
        c1 = initState;
        c2 = 0;
        c3 = 0;
        c4 = finalSetpoint;
        if(l2 > 0){
            t1 = l1;
            t2 = l1 + l2;
            t3 = l1 + l2 + l3;
            maxVelo = active.velo;
            c2 = getAccelSetpoint(t1) - getCruiseSetpoint(t1);
            c3 = getCruiseSetpoint(t2) - getDecelSetpoint(t2);
        }else{
            double a = (active.accel * active.decel - pow(active.accel, 2)) / (2 * active.decel);
            t1 = direction * sqrt(-4 * a * -distance) / (2 * a);
            t2 = t1;
            t3 = (active.decel - active.accel) * t1 / active.decel;
            maxVelo = active.accel * t1;
            c3 = getAccelSetpoint(t1) - getDecelSetpoint(t1);
        }
        timer.reset();
    }

    /**
     * Call in the loop() method of a subassembly to replace the setpoint in a PID getFeedback() call
     * @return a profiled setpoint for that particular moment in time
     */
    public double getInstantSetpoint(){
        double instantTime = timer.time();
        if(instantTime < t1){
            return getAccelSetpoint(instantTime);
        }else if(instantTime < t2){
            return getCruiseSetpoint(instantTime);
        }else if(instantTime < t3){
            return getDecelSetpoint(instantTime);
        }else{
            return c4;
        }
    }

    private double getAccelSetpoint(double instantTime){
        return (0.5 * active.accel * pow(instantTime, 2)) + c1;
    }

    private double getCruiseSetpoint(double instantTime){
        return (active.velo * instantTime) + c2;
    }

    private double getDecelSetpoint(double instantTime){
        return (0.5 * active.decel * pow(instantTime - t2, 2)) + (maxVelo * instantTime) + c3;
    }

    public String getTimes(){
        return t1 + " " + t2 + " " + t3;
    }
}