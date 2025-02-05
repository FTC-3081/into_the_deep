package org.firstinspires.ftc.teamcode.common.subassems.scoring;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.control.geometry.Range;

public class DifferentialAssem {

    private final Servo lServo, rServo;
    private final double offset1, offset2;
    private final double SPR;
    private final Range angle1Range, angle2Range;
    private final Range servoRange = new Range(0, 1);

    private double angle1, angle2;
    private double lSetpoint, rSetpoint;

    /**
     * 2 Servo Differential
     * @param lServo the left servo
     * @param rServo the right servo
     * @param offset1 the offset from the servo zero position to the zero position of the differential's 1st stage, in radians
     * @param offset2 the offset from the servo zero position to the zero position of the differential's 2nd stage, in radians
     * @param SPR Servo range per radian
     * @param angle1Range the allowable range of motion of the differential's 1st stage, in radians
     * @param angle2Range the allowable range of motion of the differential's 2nd stage, in radians
     */
    public DifferentialAssem(
            Servo lServo, Servo rServo,
            double offset1, double offset2, double SPR,
            Range angle1Range, Range angle2Range
    ){
        this.lServo = lServo;
        this.rServo = rServo;
        this.offset1 = offset1;
        this.offset2 = offset2;
        this.angle1Range = angle1Range;
        this.angle2Range = angle2Range;
        this.SPR = SPR;
    }

    public void loop(){
        double angle1Actual = angle1Range.constrain(angle1) + offset1;
        double angle2Actual = angle2Range.constrain(angle2) + offset2;
        lSetpoint = servoRange.constrain((angle1Actual - angle2Actual) * SPR);
        rSetpoint = servoRange.constrain((angle1Actual + angle2Actual) * SPR);
    }

    public void write(){
        lServo.setPosition(lSetpoint);
        rServo.setPosition(rSetpoint);
    }

    public void setPosition(double angle1, double angle2){
        this.angle1 = angle1;
        this.angle2 = angle2;
    }
}
