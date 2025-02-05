package org.firstinspires.ftc.teamcode.common.control.geometry;

public class Vector {
    public double direction, magnitude;

    /**
     * Instantiates vector object with direction and magnitude
     * @param direction theta/angle value
     * @param magnitude distance from origin
     */
    public Vector(double direction, double magnitude) {
        this.direction = direction;
        this.magnitude = magnitude;
    }

    /**
     * Instantiates vector object with all values at 0
     */
    public Vector() {
        this(0, 0);
    }

    public Point getAsPoint(){
        return new Point(
                magnitude * Math.cos(direction),
                magnitude * Math.sin(direction));
    }
}
