package org.firstinspires.ftc.teamcode.common.subassems.drive;

import static org.firstinspires.ftc.teamcode.common.Globals.MAX_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.common.Globals.MAX_TURN_SPEED;
import static org.firstinspires.ftc.teamcode.common.Globals.REMEMBRANCE_ZONE;
import static org.firstinspires.ftc.teamcode.common.Globals.USE_FIELD_CENTRIC_DRIVE;
import static org.firstinspires.ftc.teamcode.common.Globals.USE_POSITION_REMEMBRANCE;
import static org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff.absMax;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.DirectionalFF;
import org.firstinspires.ftc.teamcode.common.control.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.LinearPID;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.WraparoundPID;
import org.firstinspires.ftc.teamcode.common.customHardware.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.subassems.RobotAssem;
import org.firstinspires.ftc.teamcode.common.subassems.Subassem;

public class SwerveDrivetrain implements Subassem {

    public final SwerveModule[] modules;
    public final TwoWheelLocalizer localizer;

    public LinearPID xController;
    public LinearPID yController;
    public WraparoundPID tController;
    public DirectionalFF feedforward;

    private final Range xyAccuracyRange;
    private final Range tAccuracyRange;
    private final Range motorRange = new Range(-1, 1);

    private Pose setpoint = new Pose();

    private final int WHEEL_BASE, TRACK_WIDTH;
    private final double CROSS_DISTANCE;

    private double[] wheelAngles = new double[4];
    private double[] wheelSpeeds = new double[4];

    private double driveSpeed = MAX_DRIVE_SPEED, turnSpeed = MAX_TURN_SPEED;

    /**
     * @param WHEEL_BASE front-back module C2C
     * @param TRACK_WIDTH side-side module C2C
     */
    public SwerveDrivetrain(
            DcMotorEx lfDrive, DcMotorEx rfDrive, DcMotorEx lbDrive, DcMotorEx rbDrive,
            CRServo lfSwerve, CRServo rfSwerve, CRServo lbSwerve, CRServo rbSwerve,
            AnalogInput lfEncoder, AnalogInput rfEncoder, AnalogInput lbEncoder, AnalogInput rbEncoder,
            TwoWheelLocalizer localizer,
            LinearPID xController, LinearPID yController, WraparoundPID tController, DirectionalFF feedforward,
            Range xyAccuracyRange, Range tAccuracyRange,
            double LF_OFFSET, double RF_OFFSET, double LB_OFFSET, double RB_OFFSET,
            int WHEEL_BASE, int TRACK_WIDTH
    ){
        this.localizer = localizer;
        this.xController = xController;
        this.yController = yController;
        this.tController = tController;
        this.feedforward = feedforward;
        this.xyAccuracyRange = xyAccuracyRange;
        this.tAccuracyRange = tAccuracyRange;
        this.WHEEL_BASE = WHEEL_BASE;
        this.TRACK_WIDTH = TRACK_WIDTH;
        CROSS_DISTANCE = hypot(WHEEL_BASE, TRACK_WIDTH);
        modules = new SwerveModule[]{
                new SwerveModule("lf", lfDrive, lfSwerve, lfEncoder, LF_OFFSET),
                new SwerveModule("rf", rfDrive, rfSwerve, rfEncoder, RF_OFFSET),
                new SwerveModule("lb", lbDrive, lbSwerve, lbEncoder, LB_OFFSET),
                new SwerveModule("rb", rbDrive, rbSwerve, rbEncoder, RB_OFFSET)};
    }

    @Override
    public void read(){
        localizer.read();
        for(SwerveModule module : modules) module.read();
    }

    @Override
    public void loop(){
        loop(
                new Pose(
                        motorRange.constrain(xController.getFeedback(setpoint.x, localizer.getPosition().x)),
                        motorRange.constrain(yController.getFeedback(setpoint.y, localizer.getPosition().y)),
                        motorRange.constrain(-tController.getFeedback(setpoint.t, localizer.getPosition().t))
                )
        );
    }

    /**
     * @param drivePose x & y joystick & theta trigger input or x, y, & t PID values
     */
    public void loop(Pose drivePose){

        drivePose.multiply(driveSpeed);
        drivePose.t *= turnSpeed;
        if(USE_FIELD_CENTRIC_DRIVE) drivePose.rotate(-localizer.getPosition().t);

        double a = drivePose.x - drivePose.t * (WHEEL_BASE / CROSS_DISTANCE),
                b = drivePose.x + drivePose.t * (WHEEL_BASE / CROSS_DISTANCE),
                c = drivePose.y - drivePose.t * (TRACK_WIDTH / CROSS_DISTANCE),
                d = drivePose.y + drivePose.t * (TRACK_WIDTH / CROSS_DISTANCE);

        wheelSpeeds = new double[]{hypot(b, d), hypot(b, c), hypot(a, d), hypot(a, c)};
        if(
                Math.abs(drivePose.x) > REMEMBRANCE_ZONE ||
                Math.abs(drivePose.y) > REMEMBRANCE_ZONE ||
                Math.abs(drivePose.t) > 0 ||
                !USE_POSITION_REMEMBRANCE
        ) wheelAngles = new double[]{-atan2(b, d), -atan2(b, c), -atan2(a, d), -atan2(a, c)};

//        Cap wheel speeds to 1
        double maxWheelSpeed = Math.abs(absMax(wheelSpeeds));
        if(maxWheelSpeed > 1) for(int i = 0; i < 4; i++) wheelSpeeds[i] /= maxWheelSpeed;
//        Scale and set wheel speeds to minimum feedforward power
        double Kv = Globals.OPERATING_VOLTAGE / RobotAssem.getVoltage();
        for (int i = 0; i < 4; i++) wheelSpeeds[i] = wheelSpeeds[i] * (1 - feedforward.getKf() * Kv) + feedforward.get(wheelSpeeds[i]) * Kv;
//        Send to modules
        for(int i = 0; i < 4; i++) modules[i].loop(wheelSpeeds[i], wheelAngles[i]);
    }

    @Override
    public void write(){
        for(SwerveModule module : modules) module.write();
    }

    public void setPosition(Pose setpoint){
        this.setpoint = setpoint;
    }

    public boolean isAtPosition(){
        return
                xyAccuracyRange.contains(setpoint.x - localizer.getPosition().x) &&
                xyAccuracyRange.contains(setpoint.y - localizer.getPosition().y) &&
                tAccuracyRange.contains(setpoint.t - localizer.getPosition().t);
    }

    public void setDriveSpeed(double driveSpeed, double turnSpeed){
        this.driveSpeed = driveSpeed;
        this.turnSpeed = turnSpeed;
    }

    public String getTelemetry(){
        String telemetry = "";
        for(SwerveModule module : modules) telemetry += (module.getSignature() + ": " + module.getServoPower() + ", ");
        return telemetry;
    }
}
//        Maintain heading when not turning
//        isTurning = t != 0 || (isTurning && zGyroVelocity != 0);
//        if(isTurning) zGyroSetpoint = zGyroState;
//        else t = zGyroControl.getFeedback(zGyroSetpoint, zGyroState);
//        Update modules