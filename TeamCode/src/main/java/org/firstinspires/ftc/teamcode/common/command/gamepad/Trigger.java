package org.firstinspires.ftc.teamcode.common.command.gamepad;

public class Trigger extends Key{
    private double state = 0;

    Trigger(GamepadEx gamepad, KeyID keyID){
        super(gamepad, keyID);
    }

    void updateState(){
        state = gamepad.getTrigger(keyID);
    }

    public double getState(){
        return state;
    }
}
