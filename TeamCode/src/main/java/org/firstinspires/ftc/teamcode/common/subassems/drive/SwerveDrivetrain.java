package org.firstinspires.ftc.teamcode.common.subassems.drive;

import static org.firstinspires.ftc.teamcode.common.Globals.MAX_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.common.Globals.MAX_TURN_SPEED;
import static org.firstinspires.ftc.teamcode.common.Globals.REMEMBRANCE_ZONE;
import static org.firstinspires.ftc.teamcode.common.Globals.USE_FIELD_CENTRIC_DRIVE;
import static org.firstinspires.ftc.teamcode.common.Globals.USE_OPTIMIZED_MODULE_ROTATION;
import static org.firstinspires.ftc.teamcode.common.Globals.USE_POSITION_REMEMBRANCE;
import static org.firstinspires.ftc.teamcode.common.Globals.USE_POWER_CUTS;
import static org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff.absMax;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.signum;
import static java.lang.Math.toDegrees;

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
    private boolean usePositionRemembrance = USE_POSITION_REMEMBRANCE;
    private double remembranceZone  = REMEMBRANCE_ZONE;
    private boolean useFieldCentricDrive = USE_FIELD_CENTRIC_DRIVE;
    private boolean usePowerCuts = USE_POWER_CUTS;

    /**
     * @param WHEEL_BASE front-back module C2C
     * @param TRACK_WIDTH side-side module C2C
     */
    public SwerveDrivetrain(
            SwerveModule lfModule, SwerveModule rfModule, SwerveModule lbModule, SwerveModule rbModule,
            TwoWheelLocalizer localizer,
            LinearPID xController, LinearPID yController, WraparoundPID tController, DirectionalFF feedforward,
            Range xyAccuracyRange, Range tAccuracyRange,
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
        modules = new SwerveModule[]{lfModule, rfModule, lbModule, rbModule};
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
        if(useFieldCentricDrive) drivePose.rotate(-localizer.getPosition().t);

        double a = drivePose.x - drivePose.t * (WHEEL_BASE / CROSS_DISTANCE),
                b = drivePose.x + drivePose.t * (WHEEL_BASE / CROSS_DISTANCE),
                c = drivePose.y - drivePose.t * (TRACK_WIDTH / CROSS_DISTANCE),
                d = drivePose.y + drivePose.t * (TRACK_WIDTH / CROSS_DISTANCE);
        if(
                Math.abs(drivePose.x) > remembranceZone ||
                Math.abs(drivePose.y) > remembranceZone ||
                Math.abs(drivePose.t) > remembranceZone ||
                !usePositionRemembrance
        ) wheelAngles = new double[]{-atan2(b, d), -atan2(b, c), -atan2(a, d), -atan2(a, c)};

        if(usePowerCuts && !modulesAreAtPosition()){
            wheelSpeeds = new double[]{0, 0, 0, 0};
        }else{
            wheelSpeeds = new double[]{hypot(b, d), hypot(b, c), hypot(a, d), hypot(a, c)};
//            Cap wheel speeds to 1
            double maxWheelSpeed = Math.abs(absMax(wheelSpeeds));
            if(maxWheelSpeed > 1) for(int i = 0; i < 4; i++) wheelSpeeds[i] /= maxWheelSpeed;
//            Scale and set wheel speeds to minimum feedforward power
            double Kv = Globals.MAX_VOLTAGE / RobotAssem.getVoltage();
            for (int i = 0; i < 4; i++) wheelSpeeds[i] = wheelSpeeds[i] * (1 - feedforward.getKf() * Kv) + feedforward.get(wheelSpeeds[i]) * Kv;
        }

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

    public boolean modulesAreAtPosition(){
        boolean atPosition = true;
        for(SwerveModule module : modules) atPosition = atPosition && module.isAtPosition();
        return atPosition;
    }

    public void setDriveSpeed(double driveSpeed, double turnSpeed){
        this.driveSpeed = driveSpeed;
        this.turnSpeed = turnSpeed;
    }

    public String getTelemetry(){
        String telemetry = "";
        for(SwerveModule module : modules) telemetry += (module.getSignature() + ": " + ((int)toDegrees(module.getPosition())) + ", ");
        return telemetry;
    }

    public void useOptimizedModuleRotation(boolean useOptimizedModuleRotation){
        for (SwerveModule module : modules) module.useOptimizedModuleRotation(useOptimizedModuleRotation);
    }

    public void useFieldCentricDrive(boolean useFieldCentricDrive){
        this.useFieldCentricDrive = useFieldCentricDrive;
    }

    public void usePositionRemembrance(boolean usePositionRemembrance, double remembranceZone){
        this.usePositionRemembrance = usePositionRemembrance;
        this.remembranceZone = remembranceZone;
    }

    public void usePowerCuts(boolean usePowerCuts){
        this.usePowerCuts = usePowerCuts;
    }
}
//        Maintain heading when not turning
//        isTurning = t != 0 || (isTurning && zGyroVelocity != 0);
//        if(isTurning) zGyroSetpoint = zGyroState;
//        else t = zGyroControl.getFeedback(zGyroSetpoint, zGyroState);
//        Update modules