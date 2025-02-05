package org.firstinspires.ftc.teamcode.common.command.gamepad;

public class Button extends Key {

    private boolean state;
    private boolean prevState;

    Button(GamepadEx gamepad, KeyID keyID){
        super(gamepad, keyID);
    }

    void updateState(){
        prevState = state;
        state = gamepad.getButton(keyID);
    }

    public boolean isPressed(){
        return state;
    }

    public boolean wasJustPressed(){
        return state && !prevState;
    }

    public boolean wasJustUnpressed(){
        return prevState && !state;
    }
}
