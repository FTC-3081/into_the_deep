package org.firstinspires.ftc.teamcode.common.subassems.scoring;

import static org.firstinspires.ftc.teamcode.common.Globals.ARM_M_HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.common.Globals.ARM_T_HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.common.Globals.ARM_T_INTAKE_SPECIMEN;
import static org.firstinspires.ftc.teamcode.common.Globals.ARM_X_HIGH_BAR;
import static org.firstinspires.ftc.teamcode.common.Globals.ARM_X_HIGH_BAR_SNAP;
import static org.firstinspires.ftc.teamcode.common.Globals.ARM_X_INTAKE_SAMPLE;
import static org.firstinspires.ftc.teamcode.common.Globals.ARM_Y_HIGH_BAR;
import static org.firstinspires.ftc.teamcode.common.Globals.ARM_Y_HIGH_BAR_SNAP;
import static org.firstinspires.ftc.teamcode.common.Globals.ARM_Y_INTAKE_SAMPLE;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_HIGH_BAR_1;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_HIGH_BAR_2;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_HIGH_BASKET_1;
import static org.firstinspires.ftc.teamcode.common.Globals.DIFFY_INTAKE_SAMPLE_1;
import static org.firstinspires.ftc.teamcode.common.Globals.LEVEL_HIGH_BAR;
import static org.firstinspires.ftc.teamcode.common.Globals.LEVEL_INTAKE_SPECIMEN;
import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.common.control.geometry.Point;
import org.firstinspires.ftc.teamcode.common.control.geometry.Vector;
import org.firstinspires.ftc.teamcode.common.subassems.Subassem;

public class ScoringAssem implements Subassem{

    public final ArmAssem arm;
    public final DifferentialAssem diffy;
    public final MiniRevoluteAssem level;
    public final MiniRollerAssem roller;

    private Setpoint setpoint = Setpoint.IDLE;
    private boolean diffyLevel = false;

    public ScoringAssem(ArmAssem arm, DifferentialAssem diffy, MiniRevoluteAssem level, MiniRollerAssem roller){
        this.arm = arm;
        this.diffy = diffy;
        this.level = level;
        this.roller = roller;
    }

    @Override
    public void read() {
        arm.read();
    }

    @Override
    public void loop() {
        if(diffyLevel) diffy.setPosition(Math.PI/2 - arm.getPositionAsVector().direction, 0);
        arm.loop();
        diffy.loop();
        level.loop();
    }

    @Override
    public void write() {
        arm.write();
        diffy.write();
        level.write();
        roller.write();
    }

    public void setPosition(Setpoint setpoint){
        this.setpoint = setpoint;
        switch (setpoint){
            case IDLE:
                arm.setPosition(arm.offset);
                diffy.setPosition(0, 0);
                level.setPosition(0);
                break;
            case PRE_INTAKE:
                arm.setPosition(new Vector(0, 260.5));
                diffy.setPosition(toRadians(90), 0);
                level.setPosition(0);
                break;
            case HIGH_BASKET:
                arm.setPosition(new Vector(toRadians(ARM_T_HIGH_BASKET), ARM_M_HIGH_BASKET));
                diffy.setPosition(toRadians(DIFFY_HIGH_BASKET_1), 0);
                level.setPosition(0);
                break;
            case HIGH_BAR:
                arm.setPosition(new Point(ARM_X_HIGH_BAR, ARM_Y_HIGH_BAR));
                diffy.setPosition(toRadians(DIFFY_HIGH_BAR_1), toRadians(DIFFY_HIGH_BAR_2));
                level.setPosition(toRadians(LEVEL_HIGH_BAR));
                break;
            case HIGH_BAR_SNAP:
                arm.setPosition(new Point(ARM_X_HIGH_BAR_SNAP, ARM_Y_HIGH_BAR_SNAP));
                diffy.setPosition(toRadians(DIFFY_HIGH_BAR_1), toRadians(DIFFY_HIGH_BAR_2));
                level.setPosition(toRadians(LEVEL_HIGH_BAR));
                break;
            case INTAKE_SAMPLE:
                arm.setPosition(new Point(ARM_X_INTAKE_SAMPLE, ARM_Y_INTAKE_SAMPLE));
                diffy.setPosition(toRadians(DIFFY_INTAKE_SAMPLE_1), 0);
                level.setPosition(0);
                break;
            case INTAKE_SPECIMEN:
                arm.setPosition(new Vector(toRadians(ARM_T_INTAKE_SPECIMEN), 260.5));
                diffy.setPosition(toRadians(261 - ARM_T_INTAKE_SPECIMEN), 0);
                level.setPosition(toRadians(LEVEL_INTAKE_SPECIMEN));
                break;
        }
    }

    public Setpoint getPosition(){
        return setpoint;
    }

    public boolean isAt(Setpoint setpoint){
        return this.setpoint == setpoint;
    }

    public boolean isIntaking(){
        return setpoint == Setpoint.INTAKE_SAMPLE || setpoint == Setpoint.INTAKE_SPECIMEN;
    }

    public void setDiffyLevel(boolean diffyLevel){
        this.diffyLevel = diffyLevel;
    }

    public enum Setpoint {
        IDLE,
        PRE_INTAKE,
        HIGH_BASKET,
        LOW_BASKET,
        HIGH_BAR,
        HIGH_BAR_SNAP,
        LOW_BAR,
        INTAKE_SAMPLE,
        INTAKE_SPECIMEN
    }
}
