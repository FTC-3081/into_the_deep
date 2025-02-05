package org.firstinspires.ftc.teamcode.common.customHardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class QuadratureEncoder extends RotaryEncoder {

    private final DcMotorEx motor;

    /**
     * @param motor the motor that shares the port with the encoder
     * @param TPR ticks per revolution
     * @param EXTERNAL_REDUCTION output/input teeth
     */
    public QuadratureEncoder(DcMotorEx motor, int TPR, double EXTERNAL_REDUCTION){
        super(0, (double) 1 / TPR * 2 * Math.PI / EXTERNAL_REDUCTION);
        this.motor = motor;
    }

    @Override
    public double read(){
        position = (double) motor.getCurrentPosition() * OUTPUT_SCALAR - zeroOffset;
        return position;
    }
}