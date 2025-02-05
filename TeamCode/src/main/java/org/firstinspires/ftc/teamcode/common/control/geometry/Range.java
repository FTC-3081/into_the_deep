package org.firstinspires.ftc.teamcode.common.control.geometry;

public class Range {

    public final double max, min;

    public Range(double minmax) {
        this(-minmax, minmax);
    }

    public Range(double min, double max) {
        this.max = max;
        this.min = min;
    }

    public boolean contains(double value){
        return max > value && value > min;
    }

    public double constrain(double value){
        if(value > max) value = max;
        else if(value < min) value = min;
        return value;
    }
}
