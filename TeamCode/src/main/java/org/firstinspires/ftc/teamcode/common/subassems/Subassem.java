package org.firstinspires.ftc.teamcode.common.subassems;

public interface Subassem {

    /**
     * Reads all sensors
     * Call once per opMode loop
     */
    void read();

    /**
     * Calculates new values to write to hardware
     * Call once per opMode loop
     */
    void loop();

    /**
     * Writes new values to hardware
     * Call once per opMode loop
     */
    void write();

}
