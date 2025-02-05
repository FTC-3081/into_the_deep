package org.firstinspires.ftc.teamcode.common.control.feedforwards;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.firstinspires.ftc.teamcode.common.control.geometry.Range;

import java.util.function.DoubleSupplier;

public class PivotFF extends DirectionalFF{

    private final DoubleSupplier position;
    private final DoubleSupplier scalar;
    private final double KgMin, KgMax;

    /**
     * Represents the pivot of an arm that pivots and extends (assume 0 degrees to be up and 90 to be forward)
     * @param gravityMin the necessary motor power to start raising the arm from 90 to 0 at minimum extension
     * @param gravityMax the necessary motor power to start raising the arm from 90 to 0 at maximum extension
     * @param friction the necessary motor power to start lowering the arm from 0 to 90
     * @param position the getPosition() method for the pivot assembly
     * @param scalar the getPercentExtension() method for the extension assembly
     */
    public PivotFF(double gravityMin, double gravityMax, double friction, DoubleSupplier position, DoubleSupplier scalar, double KfDead, Range deadRange){
        super(friction, KfDead, deadRange);
        this.position = position;
        this.scalar = scalar;
        KgMin = gravityMin - friction;
        KgMax = gravityMax - gravityMin;
    }

    public double get(double error){
        return -(KgMin + KgMax * scalar.getAsDouble()) * sin(position.getAsDouble()) + super.get(error);
    }
}
