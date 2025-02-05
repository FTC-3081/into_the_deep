package org.firstinspires.ftc.teamcode.common.subassems.drive;
import static org.firstinspires.ftc.teamcode.common.Globals.USE_OPTIMIZED_MODULE_ROTATION;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.control.filters.UsefulStuff;
import org.firstinspires.ftc.teamcode.common.control.feedforwards.DirectionalFF;
import org.firstinspires.ftc.teamcode.common.control.geometry.Range;
import org.firstinspires.ftc.teamcode.common.control.pidControllers.WraparoundPID;
import org.firstinspires.ftc.teamcode.common.customHardware.AnalogEncoder;

public class SwerveModule {

    private final String signature;
    private final DcMotorEx motor;
    private final CRServo servo;
    private final AnalogEncoder encoder;
    private final Range servoRange = new Range(-1, 1);

    public WraparoundPID controller = new WraparoundPID(
            0.21, 0, 0.09, new DirectionalFF(0.1, 0.06, new Range(toRadians(7.5))), 0.95, 0
    );
//0.22
    private double state = 0;
    private double setpoint = 0;
    private double servoPower = 0;
    private double motorPower = 0;

    /**
     * @param signature a denotation of module's position on the robot for telemetry purposes
     * @param motor motor that drives the wheel
     * @param servo servo that rotates the swerve module
     * @param encoder analog encoder in the servo
     */
    public SwerveModule (String signature, DcMotorEx motor, CRServo servo, AnalogInput encoder, double OFFSET){
        this.signature = signature;
        this.motor = motor;
        this.servo = servo;
        this.encoder = new AnalogEncoder(encoder, OFFSET);
    }

    public void read(){
        state = encoder.read();
    }

    public void loop(double wheelSpeed, double wheelSetpoint){
        setpoint = wheelSetpoint;
        motorPower = wheelSpeed;
        double error = UsefulStuff.normalizeAngle(setpoint - state);
        if(USE_OPTIMIZED_MODULE_ROTATION && Math.abs(error) > Math.PI / 2){
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

    public double getSetpoint(){
        return setpoint;
    }

    public String getSignature(){
        return signature;
    }

    public double getServoPower(){
        return servoPower;
    }
}