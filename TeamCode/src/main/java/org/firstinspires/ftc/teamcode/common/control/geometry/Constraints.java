package org.firstinspires.ftc.teamcode.common.control.geometry;

import static java.lang.Math.abs;

public class Constraints {

    public final double velo, accel, decel;

    private Constraints(double velo, double accel, double decel, double scalar){
        this.velo = abs(velo) * scalar;
        this.accel = abs(accel) * scalar;
        this.decel = -abs(decel) * scalar;
    }

    public Constraints(double velo, double accel, double decel){
        this(velo, accel, decel, 1);
    }

    public Constraints(double velo, double accel){
        this(velo, accel, accel);
    }

    public Constraints reverse(){
        return new Constraints(velo, accel, decel, -1);
    }
}