package org.firstinspires.ftc.teamcode.common.command.gamepad;

abstract class Key {

    protected final GamepadEx gamepad;
    protected final KeyID keyID;

    Key(GamepadEx gamepad, KeyID keyID){
        this.gamepad = gamepad;
        this.keyID = keyID;
    }

    abstract void updateState();

    public KeyID getPosition(){
        return keyID;
    }
}
