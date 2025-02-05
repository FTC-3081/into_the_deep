package org.firstinspires.ftc.teamcode.common.subassems.scoring;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.control.motionProfiles.TrapezoidMP;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.LinearPID;
import org.firstinspires.ftc.teamcode.common.customHardware.RotaryEncoder;
import org.firstinspires.ftc.teamcode.common.subassems.Subassem;

public class RevoluteAssem implements Subassem {

    protected final DcMotorEx[] motors;
    protected final RotaryEncoder encoder;

    protected final ElapsedTime timer = new ElapsedTime();

    public LinearPID[] controllers = new LinearPID[2];
    public TrapezoidMP profiler;
    protected int activeController = 0;
    protected boolean enableMP = false;

    protected final Range positionRange;
    protected final Range accuracyRange;
    protected final Range velocityRange;
    protected final Range motorRange = new Range(-1, 1);

    protected double state = 0;
    protected double prevState = 0;
    protected double velocity = 0;
    protected double setpoint = 0;
    protected double power = 0;
    protected double overridePower = 0;

    /**
     * Revolute assembly with an alternate PID controller and a motion profile
     * @param controller1 any PID controller
     * @param controller2 an alternate PID controller
     * @param profiler any motion profile, units in radians/second (squared)
     * @param positionRange the minimum and maximum positions in radians
     * @param accuracyRange the allowable error to be considered at position in radians
     * @param velocityRange the allowable error in velocity to be considered not moving in radians
     * @param encoder encoder (of the output shaft)
     * @param motors all motors driving the system
     */
    public RevoluteAssem(
            LinearPID controller1, LinearPID controller2, TrapezoidMP profiler,
            Range positionRange, Range accuracyRange, Range velocityRange,
            RotaryEncoder encoder, DcMotorEx... motors
    ){
        controllers[0] = controller1;
        controllers[1] = controller2;
        this.profiler = profiler;
        this.positionRange = positionRange;
        this.accuracyRange = accuracyRange;
        this.velocityRange = velocityRange;
        this.encoder = encoder;
        this.motors = motors;
        enableMP = true;
    }

    /**
     * Revolute assembly with a motion profile
     * @param controller any PID controller
     * @param profiler any motion profile, units in radians/second (squared)
     * @param positionRange the minimum and maximum positions in radians
     * @param accuracyRange the allowable error to be considered at position in radians
     * @param velocityRange the allowable error in velocity to be considered not moving in radians
     * @param encoder encoder (of the output shaft)
     * @param motors all motors driving the system
     */
    public RevoluteAssem(
            LinearPID controller, TrapezoidMP profiler,
            Range positionRange, Range accuracyRange, Range velocityRange,
            RotaryEncoder encoder, DcMotorEx... motors
    ){
        this(controller, controller, profiler, positionRange, accuracyRange, velocityRange, encoder, motors);
    }

    /**
     * Revolute assembly with an alternate PID controller
     * @param controller1 any PID controller
     * @param controller2 an alternate PID controller
     * @param positionRange the minimum and maximum positions in radians
     * @param accuracyRange the allowable error to be considered at position in radians
     * @param velocityRange the allowable error in velocity to be considered not moving in radians
     * @param encoder encoder (of the output shaft)
     * @param motors all motors driving the system
     */
    public RevoluteAssem(
            LinearPID controller1, LinearPID controller2,
            Range positionRange, Range accuracyRange, Range velocityRange,
            RotaryEncoder encoder, DcMotorEx... motors
    ){
        controllers[0] = controller1;
        controllers[1] = controller2;
        this.positionRange = positionRange;
        this.accuracyRange = accuracyRange;
        this.velocityRange = velocityRange;
        this.encoder = encoder;
        this.motors = motors;
    }

    /**
     * Revolute assembly with nothing special
     * @param controller any PID controller
     * @param positionRange the minimum and maximum positions in radians
     * @param accuracyRange the allowable error to be considered at position in radians
     * @param velocityRange the allowable error in velocity to be considered not moving in radians
     * @param encoder encoder (of the output shaft)
     * @param motors all motors driving the system
     */
    public RevoluteAssem(
            LinearPID controller,
            Range positionRange, Range accuracyRange, Range velocityRange,
            RotaryEncoder encoder, DcMotorEx... motors
    ){
        this(controller, controller, positionRange, accuracyRange, velocityRange, encoder, motors);
    }

    @Override
    public void read() {
        state = UsefulStuff.normalizeAngle(encoder.read());
        velocity = (state - prevState) / timer.time();
        timer.reset();
        prevState = state;
    }

    @Override
    public void loop() {
        power = motorRange.constrain(controllers[activeController].getFeedback(enableMP ? profiler.getInstantSetpoint() : setpoint, state) + overridePower);
    }

    @Override
    public void write() {
        for(DcMotorEx motor : motors) motor.setPower(power);
    }

    /**
     * Changes the setpoint of the assembly, does not make hardware call
     * @param setpoint the new setpoint, in millimeters or radians
     */
    public void setPosition(double setpoint){
        if(enableMP) profiler.setProfile(state, setpoint);
        this.setpoint = positionRange.constrain(setpoint);
    }

    public double getPosition(){
        return state;
    }

    public double getVelocity(){
        return velocity;
    }

    public void setController(int active){
        this.activeController = active;
    }

    public boolean isAtPosition(){
        return accuracyRange.contains(setpoint - state);
    }

    public boolean isStopped(){
        return velocityRange.contains(velocity);
    }

    public void enableMP(boolean enableMP){
        if(profiler != null) this.enableMP = enableMP;
    }

    public void zero(){
        encoder.zero();
    }

    public void overridePID(double overridePower){
        this.overridePower = overridePower;
    }
}
