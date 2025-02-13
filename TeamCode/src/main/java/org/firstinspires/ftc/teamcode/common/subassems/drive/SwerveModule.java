package org.firstinspires.ftc.teamcode.common.subassems.drive;
import static org.firstinspires.ftc.teamcode.common.Globals.USE_OPTIMIZED_MODULE_ROTATION;
import static org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff.normalizeAngle;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.DirectionalFF;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.WraparoundPID;
import org.firstinspires.ftc.teamcode.common.customHardware.AnalogEncoder;
import org.firstinspires.ftc.teamcode.opMode.teleop.Teleop_v4_0;

public class SwerveModule {

    private final String signature;
    private final DcMotorEx motor;
    private final CRServo servo;
    private final AnalogEncoder encoder;

    private final Range accuracyRange;
    private final Range servoRange = new Range(-1, 1);

    public WraparoundPID controller;

    private double state = 0;
    private double setpoint = 0;
    private double servoPower = 0;
    private double motorPower = 0;

    private boolean useOptimizedModuleRotation = USE_OPTIMIZED_MODULE_ROTATION;

    /**
     * @param signature a denotation of module's position on the robot for telemetry purposes
     * @param motor motor that drives the wheel
     * @param servo servo that rotates the swerve module
     * @param encoder analog encoder in the servo
     */
    public SwerveModule (String signature, DcMotorEx motor, CRServo servo, AnalogInput encoder, WraparoundPID controller, Range accuracyRange, double OFFSET){
        this.signature = signature;
        this.motor = motor;
        this.servo = servo;
        this.encoder = new AnalogEncoder(encoder, OFFSET);
        this.controller = controller;
        this.accuracyRange = accuracyRange;
    }

    public void read(){
        state = encoder.read();
    }

    public void loop(double wheelSpeed, double wheelSetpoint){
        setpoint = wheelSetpoint;
        motorPower = wheelSpeed;
        double error = normalizeAngle(setpoint - state);
        if(useOptimizedModuleRotation && Math.abs(error) > Math.PI / 2){
            setpoint -= Math.PI;
            motorPower *= -1;
        }
        servoPower = servoRange.constrain(controller.getFeedback(setpoint, state));
    }

    public void write(){
        servo.setPower(servoPower);
        motor.setPower(motorPower);
    }

    public double getPosition(){
        return state;
    }

    public double getError(){
        return normalizeAngle(setpoint - state);
    }

    public String getSignature(){
        return signature;
    }

    public double getServoPower(){
        return servoPower;
    }

    public double getMotorPower(){
        return motorPower;
    }

    public boolean isAtPosition(){
        return accuracyRange.contains(getError());
    }

    public void useOptimizedModuleRotation(boolean useOptimizedModuleRotation){
        this.useOptimizedModuleRotation = useOptimizedModuleRotation;
    }
}