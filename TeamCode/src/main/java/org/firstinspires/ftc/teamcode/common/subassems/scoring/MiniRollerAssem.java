package org.firstinspires.ftc.teamcode.common.subassems.scoring;

import com.qualcomm.robotcore.hardware.CRServo;

public class MiniRollerAssem {

    private final CRServo[] servos;

    private double power;

    public MiniRollerAssem(CRServo... servos){
        this.servos = servos;
    }

    public void write(){
        for(CRServo servo : servos){
            servo.setPower(power);
        }
    }

    public void setPower(double power){
        this.power = power;
    }

    public void scarf(){
        setPower(1);
    }

    public void stop(){
        setPower(0);
    }

    public void barf(){
        setPower(-1);
    }

}
