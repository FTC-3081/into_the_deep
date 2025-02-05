package org.firstinspires.ftc.teamcode.common.control.feedforwards;

public class ConstantFF {

    protected double Kf;

    /**
     * Represents a mechanism in which a constant force in the same direction must always be applied
     * @param Kf the necessary motor power
     */
    public ConstantFF(double Kf){
        set(Kf);
    }

    public void set(double Kf){
        this.Kf = Kf;
    }

    public double get(double error){
        return Kf;
    }
}
