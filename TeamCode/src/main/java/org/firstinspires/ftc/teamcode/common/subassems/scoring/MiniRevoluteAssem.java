package org.firstinspires.ftc.teamcode.common.subassems.scoring;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.control.geometry.Range;

public class MiniRevoluteAssem {

    private final Servo[] servos;
    private final double offset;
    private final double SPR;
    private final Range angleRange;
    private final Range servoRange = new Range(0, 1);

    private double angle;
    private double setpoint;

    public MiniRevoluteAssem(double offset, double SPR, Range angleRange, Servo... servos){
        this.offset = offset;
        this.SPR = SPR;
        this.angleRange = angleRange;
        this.servos = servos;
    }

    public void loop(){
        setpoint = servoRange.constrain((angleRange.constrain(angle) + offset) * SPR);
    }

    public void write(){
        for(Servo servo : servos){
            servo.setPosition(setpoint);
        }
    }

    public void setPosition(double angle){
        this.angle = angle;
    }
}
