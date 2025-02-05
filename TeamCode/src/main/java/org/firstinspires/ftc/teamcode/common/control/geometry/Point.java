package org.firstinspires.ftc.teamcode.common.control.geometry;

public class Point {
    public double x, y;

    /**
     * @param x side-side component
     * @param y forward-back component
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point() {
        this(0, 0);
    }

    public Vector getAsVector(){
        return new Vector(
                Math.atan2(y, x),
                Math.hypot(y, x));
    }

    /**
     * Changes all values of this point by input point
     *
     * @param point point to be added
     */
    public void add(Point point) {
        x += point.x;
        y += point.y;
    }

    /**
     * Rotates the point about the origin by angle
     * @param angle the angle by which to rotate the point
     */
    public void rotate(double angle) {
        double temp = x * Math.cos(angle) - y * Math.sin(angle);
        y = x * Math.sin(angle) + y * Math.cos(angle);
        x = temp;
    }

    public void multiply(double scalar){
        x *= scalar;
        y *= scalar;
    }

    public static Point add(Point point1, Point point2){
        return new Point(point1.x + point2.x, point1.y + point2.y);
    }

    public static Point multiply(Point point, double scalar){
        return new Point(point.x * scalar, point.y * scalar);
    }
}
