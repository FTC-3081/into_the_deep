package org.firstinspires.ftc.teamcode.common.control.filters;

// IDK I think this was my original odometry localizer but then at some point after I ditched it there were still some useful methods so I renamed it useful stuff

public class UsefulStuff {

//    Circumference of odometry wheel in MM (wheel in my design (35mm rotacastor) = 35 * Math.PI)
    private final double wheelCircumferenceMM;

//    Counts in one rotation of odometry en
//    coder (encoder in my design (Rev Thru Bore Encoder) = 8192)
    private final double countsPerRev;

//    x, y, and theta delta-sigmas representing the pose of the robot relative to its start
    private double pose_x;
    private double pose_y;
    private double pose_t;

//    Previous x, y, and theta sensor readings for calculating deltas
    private int xCountsPrev;
    private int yCountsPrev;
    private double tPrev;

    /**
     * Constructs pose object with wheel diameter 35mm and counts per rev 8192
     * (same as my odometry module design)
     * @param xEncoderStart Reading from the x encoder on init
     * @param yEncoderStart Reading from the y encoder on init
     */

    public UsefulStuff(int xEncoderStart, int yEncoderStart){
        this(35, 8192, xEncoderStart, yEncoderStart);
    }

    /**
     * @param wheelDiameterMM Diameter of odometry wheel in MM
     * @param countsPerRev Counts in one revolution of odometry encoder
     * @param xEncoderStart Reading from the x encoder on init
     * @param yEncoderStart Reading from the y encoder on init
     */

    public UsefulStuff(double wheelDiameterMM, double countsPerRev,
                       int xEncoderStart, int yEncoderStart){
        this.wheelCircumferenceMM = wheelDiameterMM * Math.PI;
        this.countsPerRev = countsPerRev;
        xCountsPrev = xEncoderStart;
        yCountsPrev = yEncoderStart;
        tPrev = 0;
    }

    /**
     * Updates the pose of the robot
     * @param xCountsCurrent The current x reading in encoder counts
     * @param yCountsCurrent The current y reading in encoder counts
     * @param tCurrent The current theta reading in radians
     */

    public void update(int xCountsCurrent, int yCountsCurrent, double tCurrent){
//        Calculate pose deltas
        double delta_x = countsToMM(xCountsCurrent - xCountsPrev);
        double delta_y = countsToMM(yCountsCurrent - yCountsPrev);
        double delta_t = tCurrent - tPrev;
//        Rotate lateral deltas by current heading with rotation matrix, add deltas to their delta-sigmas
        pose_x += rotateVector_x(delta_x, delta_y, tCurrent);
        pose_y += rotateVector_y(delta_x, delta_y, tCurrent);
        pose_t += delta_t;
//        Update previous readings to current readings for delta calculations in the next call
        xCountsPrev = xCountsCurrent;
        yCountsPrev = yCountsCurrent;
        tPrev = tCurrent;
    }

    /**
     * @return The current x (side-to-side) position of the robot in mm (right = positive)
     */

    public double getPose_x(){
        return pose_x;
    }

    /**
     * @return The current y (forward-back) position of the robot in mm (forward = positive)
     */

    public double getPose_y(){
        return pose_y;
    }

    /**
     * @return The current normalized (between -PI and +PI) theta position (rotation) of the robot in radians
     */

    public double getPose_t(){
        return normalizeAngle(pose_t);
    }

    /**
     * @param counts A number of counts from the encoder
     * @return The distance in mm indicated by the counts
     */

    public double countsToMM(int counts){
        return counts / countsPerRev * wheelCircumferenceMM;
    }

    /**
     * @param mm A distance in mm
     * @return The number of counts necessary to travel the given number of mm
     */

    public int mmToCounts(double mm){
        return (int)(mm / (wheelCircumferenceMM) * countsPerRev + 0.5);
    }

    /**
     * @param angle An angle in radians
     * @return The angle normalized between -PI/2 and +PI/2
     */
    public static double normalizeAngle(double angle){
        while(angle > Math.PI) angle -= 2 * Math.PI;
        while(angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static double max(double[] nums){
        double max = nums[0];
        for(int i = 1; i < nums.length; i++) if(nums[i] > max) max = nums[i];
        return max;
    }

    public static double absMax(double[] nums){
        double max = nums[0];
        for(int i = 1; i < nums.length; i++) if(Math.abs(nums[i]) > Math.abs(max)) max = nums[i];
        return max;
    }

    /**
     * @param x The x component of a vector in the xy plane
     * @param y The y component of a vector in the xy plane
     * @param t The angle (in radians) by which to rotate the given vector
     * @return The x component of the rotated vector
     */

    private static double rotateVector_x(double x, double y, double t){
        return x * Math.cos(t) - y * Math.sin(t);
    }

    /**
     * @param x The x component of a vector in the xy plane
     * @param y The y component of a vector in the xy plane
     * @param t The angle (in radians) by which to rotate the given vector
     * @return The y component of the rotated vector
     */

    private static double rotateVector_y(double x, double y, double t){
        return x * Math.sin(t) + y * Math.cos(t);
    }

    /**
     * Rotates the desired pose coordinates about the actual pose coordinates by the current theta value.
     * This tricks the PID controllers into generating the proper powers when the robot is not facing the desired heading
     * @param xSetpoint The desired x-coordinate
     * @param ySetpoint The desired y-coordinate
     * @return The rotated x-coordinate
     */

    public double getFalseSetpoint_x(double xSetpoint, double ySetpoint){
//        Create error vector
        double xError = pose_x - xSetpoint;
        double yError = pose_y - ySetpoint;
//        Rotate error vector by theta and subtract x error from setpoint to create false setpoint
        return pose_x - rotateVector_x(xError, yError, -pose_t);
    }

    /**
     * Rotates the desired pose coordinates about the actual pose coordinates by the current theta value.
     * This tricks the PID controllers into generating the proper powers when the robot is not facing the desired heading
     * @param xSetpoint The desired x-coordinate
     * @param ySetpoint The desired y-coordinate
     * @return The rotated y-coordinate
     */

    public double getFalseSetpoint_y(double xSetpoint, double ySetpoint){
        //        Create error vector
        double xError = pose_x - xSetpoint;
        double yError = pose_y - ySetpoint;
//        Rotate error vector by theta and subtract x error from setpoint to create false setpoint
        return pose_y - rotateVector_y(xError, yError, -pose_t);
    }
}