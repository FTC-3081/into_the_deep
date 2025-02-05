package org.firstinspires.ftc.teamcode.common.control.feedforwards;

import static java.lang.Math.signum;

import org.firstinspires.ftc.teamcode.common.control.geometry.Range;

public class DirectionalFF extends ConstantFF {

    private double KfOffset, KfDead;
    private Range deadRange;

    /**
     * Represents a vertical extension in which different forces must be applied to overcome gravity going up and friction coming down
     * @param up the necessary motor power to start moving the system up
     * @param down the necessary motor power to start moving the system down
     */
    public DirectionalFF(double up, double down, double KfDead, Range deadRange){
        super(up - (up + down) / 2);
        KfOffset = (up + down) / 2;
        this.KfDead = KfDead;
        this.deadRange = deadRange;
    }

    /**
     * Represents a mechanism in which a constant force must be applied in the direction of motion to overcome friction
     * @param friction the necessary motor power to start moving the system
     */
    public DirectionalFF(double friction, double KfDead, Range deadRange){
        this(friction, -friction, KfDead, deadRange);
    }

    public void set(double up, double down){
        KfOffset = (up + down) / 2;
        Kf = up - KfOffset;
    }

    public void set(double friction){
        this.set(friction, -friction);
    }

    public void set(double KfDead, Range deadRange){
        this.KfDead = KfDead;
        this.deadRange = deadRange;
    }

    public double get(double error){
        return (Kf - (deadRange.contains(error)? KfDead : 0)) * signum(error) + KfOffset;
    }

    public double getKf(){
        return Kf;
    }
}