package org.firstinspires.ftc.teamcode.common.command.gamepad;

import org.firstinspires.ftc.teamcode.common.control.geometry.Point;

public class Joystick extends Key{
    Point state = new Point();
    Point prevState = new Point();

    Joystick(GamepadEx gamepad, KeyID keyID){
        super(gamepad, keyID);
    }

    void updateState(){
        prevState = state;
        state = gamepad.getJoystick(keyID);
    }

    public double getX(){
        return state.x;
    }

    public double getY(){
        return state.y;
    }

    public double getDirection(){
        return state.getAsVector().direction;
    }

    public double getMagnitude(){
        return state.getAsVector().magnitude;
    }

    public boolean xWasJustPressed(){
        return state.x != 0 && prevState.x == 0;
    }

    public boolean yWasJustPressed(){
        return state.y != 0 && prevState.y == 0;
    }

    public boolean xWasJustUnpressed(){
        return state.x == 0 && prevState.x != 0;
    }

    public boolean yWasJustUnpressed(){
        return state.y == 0 && prevState.y != 0;
    }
}
