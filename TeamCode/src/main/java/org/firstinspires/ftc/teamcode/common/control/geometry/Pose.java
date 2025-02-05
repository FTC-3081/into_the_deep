package org.firstinspires.ftc.teamcode.common.control.geometry;

public class Pose extends Point{

     public double t;

    /**
     * Instantiates pose object with x, y, and theta
     * @param x side-side position
     * @param y forward-back position
     * @param t heading
     */
     public Pose(double x, double y, double t){
         super(x, y);
         this.t = t;
     }

    /**
     * Instantiates pose object with x and y
     * @param x side-side position
     * @param y forward-back position
     */
     public Pose(double x, double y){
         this(x, y, 0);
     }

    /**
     * Instantiates pose object with all values at 0
     */
    public Pose(){
         this(0, 0);
     }

    /**
     *  Changes all values of this pose by input pose
     * @param pose pose to be added
     */
    public void add(Pose pose){
        x += pose.x;
        y += pose.y;
        t += pose.t;
     }

     public void subtract(Pose pose){
         x -= pose.x;
         y -= pose.y;
         t -= pose.t;
     }

     public static Pose add(Pose pose1, Pose pose2){
        return new Pose(pose1.x + pose2.x,
                pose1.y + pose2.y,
                pose1.t + pose2.t);
     }

    public static Pose subtract(Pose pose1, Pose pose2){
        return new Pose(pose1.x - pose2.x,
                pose1.y - pose2.y,
                pose1.t - pose2.t);
    }

    public static Pose rotate(Pose pose, double angle){
        pose.rotate(angle);
        return pose;
    }
}
