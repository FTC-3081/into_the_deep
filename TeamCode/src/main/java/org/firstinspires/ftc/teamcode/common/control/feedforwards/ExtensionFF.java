package org.firstinspires.ftc.teamcode.common.control.feedforwards;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.firstinspires.ftc.teamcode.common.control.geometry.Range;

import java.util.function.DoubleSupplier;

public class ExtensionFF extends DirectionalFF{

    private final DoubleSupplier position;
    private final DoubleSupplier scalar;
    private final double KfHMin, KfHMax, KfV, Kg;

    /**
     * Represents the extension of an arm that pivots and extends (assume 0 degrees to be up and 90 to be forward)
     * @param frictionMin the necessary motor power to start extending the arm from minimum extension at 90
     * @param frictionMax the necessary motor power to start retracting the arm from maximum extension at 90
     * @param gravityUp the necessary motor power to start raising the arm at 0
     * @param gravityDown the necessary motor power to start retracting the arm at 0
     * @param position the getPosition() method for the pivot assembly
     * @param scalar the getPercentExtension() method for the extension assembly
     */
    public ExtensionFF(double frictionMin, double frictionMax, double gravityUp, double gravityDown, DoubleSupplier position, DoubleSupplier scalar, double KfDead, Range deadRange){
        super(frictionMin, KfDead, deadRange);
        this.position = position;
        this.scalar = scalar;
        Kg = (gravityUp + gravityDown) / 2;
        KfV = gravityUp - Kg;
        KfHMin = frictionMin - KfV;
        KfHMax = frictionMax - frictionMin;
    }

    public double get(double error){
        set(KfV + ((KfHMin + (KfHMax * scalar.getAsDouble())) * abs(sin(position.getAsDouble()))));
        return Kg * cos(position.getAsDouble()) + super.get(error);
    }
}
