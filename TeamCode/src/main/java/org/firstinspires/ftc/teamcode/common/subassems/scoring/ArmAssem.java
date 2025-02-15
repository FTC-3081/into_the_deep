package org.firstinspires.ftc.teamcode.common.subassems.scoring;

import static java.lang.Math.toDegrees;

import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.control.geometry.Point;
import org.firstinspires.ftc.teamcode.common.control.geometry.Vector;
import org.firstinspires.ftc.teamcode.common.subassems.Subassem;

public class ArmAssem implements Subassem {

    public final ExtensionAssem telescope;
    public final RevoluteAssem pivot;
    public final Vector offset; //29.725, 260.5;

    private Vector state;
    private Vector setpoint;

    public ArmAssem(ExtensionAssem telescope, RevoluteAssem pivot, Vector offset){
        this.telescope = telescope;
        this.pivot = pivot;
        this.offset = offset;
        setpoint = offset;
    }

    @Override
    public void read() {
        telescope.read();
        pivot.read();
        state = new Vector(pivot.getPosition() + offset.direction, telescope.getPosition() + offset.magnitude);
    }

    @Override
    public void loop() {
        telescope.loop();
        pivot.loop();
    }

    @Override
    public void write() {
        telescope.write();
        pivot.write();
    }

    public void setPosition(Point setpoint){
        this.setpoint = setpoint.getAsVector();
        setPosition();
    }

    public void setPosition(Vector setpoint){
        this.setpoint = setpoint;
        setPosition();
    }

    private void setPosition(){
        telescope.setPosition(setpoint.magnitude - offset.magnitude);
        pivot.setPosition(setpoint.direction - offset.direction);
    }

    public boolean isAtPosition(){
        return telescope.isAtPosition() && pivot.isAtPosition();
    }

    public boolean isStopped(){
        return telescope.isStopped() && pivot.isStopped();
    }

    public Point getPositionAsPoint(){
        return state.getAsPoint();
    }

    public Vector getPositionAsVector(){
        return state;
    }

    public void zero(){
        pivot.read();
        telescope.read();
        telescope.zero();
    }

    public String getTelemetryAsVector(){
        return "(" + toDegrees(getPositionAsVector().direction) + ", " + getPositionAsVector().magnitude + ")";
    }

    public String getTelemetryAsPoint(){
        return "(" + getPositionAsPoint().x + ", " + getPositionAsPoint().y + ")";
    }

    public void setManual(boolean manual){
        telescope.setManual(manual);
        pivot.setManual(manual);
    }
}
