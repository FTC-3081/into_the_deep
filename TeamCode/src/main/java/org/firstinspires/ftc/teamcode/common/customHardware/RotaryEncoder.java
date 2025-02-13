package org.firstinspires.ftc.teamcode.common.customHardware;

public abstract class RotaryEncoder {

    protected double position;
    protected double zeroOffset;

    protected final double OUTPUT_SCALAR;

    protected RotaryEncoder(double zeroOffset, double OUTPUT_SCALAR){
        this.zeroOffset = zeroOffset;
        this.OUTPUT_SCALAR = OUTPUT_SCALAR;
    }

    /**
     * makes a hardware call  and reads the current position of the encoder
     * @return the encoder position, in radians
     */
    public abstract double read();

    /**
     * returns the encoder's last read position without making hardware call
     * @return the encoder position, in radians
     */
    public double getPosition(){
        return position - zeroOffset;
    }

    /**
     * sets the current position to the zero position
     */
    public void zero(){
        zeroOffset = position;
    }
}
