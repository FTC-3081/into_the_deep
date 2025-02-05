package org.firstinspires.ftc.teamcode.common.subassems.scoring;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.control.motionProfiles.TrapezoidMP;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.LinearPID;
import org.firstinspires.ftc.teamcode.common.customHardware.QuadratureEncoder;
import org.firstinspires.ftc.teamcode.common.subassems.Subassem;

import java.util.function.DoubleSupplier;

public class ExtensionAssem extends RevoluteAssem implements Subassem{

    private final double MPR;
    private double initEncoderOffset;
    private final DoubleSupplier encoderOffset;

    /**
     * Extension assembly with an alternate PID controller and a motion profile
     * @param controller1 any PID controller
     * @param controller2 an alternate PID controller
     * @param profiler any motion profile, units in millimeters/second (squared)
     * @param positionRange the minimum and maximum positions
     * @param accuracyRange the allowable error to be considered at position in millimeters
     * @param MPR millimeters per revolution (of the output shaft)
     * @param encoder encoder (of the output shaft)
     * @param motors all motors driving the system
     */
    public ExtensionAssem(
            LinearPID controller1, LinearPID controller2, TrapezoidMP profiler,
            Range positionRange, Range accuracyRange, Range velocityRange, double MPR,
            DoubleSupplier encoderOffset, QuadratureEncoder encoder, DcMotorEx... motors
    ){
        super(controller1, controller2, profiler, positionRange, accuracyRange, velocityRange, encoder, motors);
        this.MPR = MPR / 2 / Math.PI; // coverts MPR where R is revolutions to MPR where R is radians
        this.encoderOffset = encoderOffset;
    }

    /**
     * Extension assembly with a motion profile
     * @param controller any PID controller
     * @param profiler any motion profile, units in millimeters/second (squared)
     * @param positionRange the minimum and maximum positions in millimeters
     * @param accuracyRange the allowable error to be considered at position in millimeters
     * @param MPR millimeters per revolution (of the output shaft)
     * @param encoder encoder (of the output shaft)
     * @param motors all motors driving the system
     */
    public ExtensionAssem(
            LinearPID controller, TrapezoidMP profiler,
            Range positionRange, Range accuracyRange, Range velocityRange, double MPR,
            DoubleSupplier encoderOffset, QuadratureEncoder encoder, DcMotorEx... motors
    ){
        this(controller, controller, profiler, positionRange, accuracyRange, velocityRange, MPR, encoderOffset, encoder, motors);
    }

    /**
     * Extension assembly with an alternate PID controller
     * @param controller1 any PID controller
     * @param controller2 an alternate PID controller
     * @param positionRange the minimum and maximum positions
     * @param accuracyRange the allowable error to be considered at position in millimeters
     * @param MPR millimeters per revolution (of the output shaft)
     * @param encoder encoder (of the output shaft)
     * @param motors all motors driving the system
     */
    public ExtensionAssem(
            LinearPID controller1, LinearPID controller2,
            Range positionRange, Range accuracyRange, Range velocityRange, double MPR,
            DoubleSupplier encoderOffset, QuadratureEncoder encoder, DcMotorEx... motors
    ){
        super(controller1, controller2, positionRange, accuracyRange, velocityRange, encoder, motors);
        this.MPR = MPR / 2 / Math.PI; // coverts MPR where R is revolutions to MPR where R is radians
        this.encoderOffset = encoderOffset;
    }

    /**
     * Extension assembly with nothing special
     * @param controller any PID controller
     * @param positionRange the minimum and maximum positions in millimeters
     * @param accuracyRange the allowable error to be considered at position in millimeters
     * @param MPR millimeters per revolution (of the output shaft)
     * @param encoder encoder (of the output shaft)
     * @param motors all motors driving the system
     */
    public ExtensionAssem(
            LinearPID controller,
            Range positionRange, Range accuracyRange, Range velocityRange, double MPR,
            DoubleSupplier encoderOffset, QuadratureEncoder encoder, DcMotorEx... motors
    ){
        this(controller, controller, positionRange, accuracyRange, velocityRange, MPR, encoderOffset, encoder, motors);
    }

    @Override
    public void read() {
        state = (encoder.read() - encoderOffset.getAsDouble() + initEncoderOffset) * MPR;
    }

    public double getPercentExtension(){
        return getPosition() / positionRange.max;
    }

    public void setInitEncoderOffset(double initEncoderOffset){
        this.initEncoderOffset = initEncoderOffset;
    }
}
