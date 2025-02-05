package org.firstinspires.ftc.teamcode.common.customHardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.control.filters.LowPassFilter;
public class AnalogEncoder extends RotaryEncoder{

    private final AnalogInput analogInput;
    private final LowPassFilter filter;

    /**
     * @param analogInput the analog input hardware object
     * @param VPR volts per revolution
     * @param EXTERNAL_REDUCTION output/input teeth
     * @param zeroOffset the zero offset, in radians
     */
    public AnalogEncoder(AnalogInput analogInput, double VPR, double EXTERNAL_REDUCTION, double zeroOffset, double a){
        super(zeroOffset, 1 / VPR * 2 * Math.PI / EXTERNAL_REDUCTION);
        this.analogInput = analogInput;
        filter = new LowPassFilter(a);
    }

    /**
     * @param analogInput the analog input hardware object
     * @param zeroOffset the zero offset, in radians
     */
    public AnalogEncoder(AnalogInput analogInput, double zeroOffset){
        this(analogInput, 3.3, 1, zeroOffset, 0);
    }

    /**
     * @param analogInput the analog input hardware object
     */
    public AnalogEncoder(AnalogInput analogInput){
        this(analogInput, 0);
    }

    @Override
    public double read(){
        position = filter.loop(analogInput.getVoltage() * OUTPUT_SCALAR - zeroOffset);
        return position;
    }
}