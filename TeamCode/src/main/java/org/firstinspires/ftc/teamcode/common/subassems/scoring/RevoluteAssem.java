package org.firstinspires.ftc.teamcode.common.subassems.scoring;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.control.filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.control.motionProfiles.TrapezoidMP;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.LinearPID;
import org.firstinspires.ftc.teamcode.common.customHardware.RotaryEncoder;
import org.firstinspires.ftc.teamcode.common.subassems.Subassem;

public class RevoluteAssem implements Subassem {

    protected final DcMotorEx[] motors;
    protected final RotaryEncoder encoder;

    protected final ElapsedTime velocityTimer = new ElapsedTime();
    protected final ElapsedTime setpointTimer = new ElapsedTime();
    protected final ElapsedTime stopTimer = new ElapsedTime();

    public LinearPID[] controllers = new LinearPID[2];
    public TrapezoidMP profiler;
    protected int activeController = 0;
    protected boolean enableMP = false;
    protected boolean isRetracting = false;
    protected boolean manual = false;

    protected final Range positionRange;
    protected final Range accuracyRange;
    protected final Range velocityRange;
    protected Range powerRange = new Range(-1, 1);

    protected final LowPassFilter velocityFilter = new LowPassFilter(0.75);

    protected double state = 0;
    protected double prevState = 0;
    protected double velocity = 0;
    protected double setpoint = 0;
    protected double prevSetpoint = 0;
    protected double power = 0;
    protected double current = 0;

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
    }

    protected void readCommon(){
        velocity = velocityFilter.loop((state - prevState) / velocityTimer.time());
        velocityTimer.reset();
        prevState = state;
        if(!velocityRange.contains(velocity)) stopTimer.reset();
        current = 0;
        for(DcMotorEx motor : motors){
            current += motor.getCurrent(CurrentUnit.MILLIAMPS);
        }
        current /= motors.length;
    }

    @Override
    public void loop() {
        if(!manual) power = powerRange.constrain(controllers[activeController].getFeedback(enableMP ? profiler.getInstantSetpoint() : setpoint, state));
        else if(power == 0) power = powerRange.constrain(controllers[activeController].feedforward.get(0));
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
        prevSetpoint = this.setpoint;
        if(enableMP) profiler.setProfile(state, setpoint);
        this.setpoint = positionRange.constrain(setpoint);
        setpointTimer.reset();
    }

    public double getPosition(){
        return state;
    }

    public double getVelocity(){
        return velocity;
    }

    public double getCurrent(){
        return current;
    }

    public void setController(int active){
        this.activeController = active;
    }

    public boolean isAtPosition(){
        return accuracyRange.contains(setpoint - state);
    }

    public boolean isAtPosition(double testSetpoint){
        return accuracyRange.contains(testSetpoint - state);
    }

    public boolean isStopped(){
        return stopTimer.time() > 0.1 && setpointTimer.time() > 0.1;
    }

    public void enableMP(boolean enableMP){
        if(profiler != null) this.enableMP = enableMP;
    }

    public void zero(){
        encoder.zero();
    }

    public void setPowerRange(Range powerRange){
        this.powerRange = powerRange;
    }

    public void resetPowerRange(){
        setPowerRange(new Range(-1, 1));
    }

    public void setManual(boolean manual){
        this.manual = manual;
    }

    public void setPower(double power){
        this.power = power;
    }
}
