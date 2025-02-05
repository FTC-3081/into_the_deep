package org.firstinspires.ftc.teamcode.common.command.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.control.geometry.Point;

public class GamepadEx {

    private final Gamepad gamepad;

    public final Button
            A = new Button(this, KeyID.A),
            B = new Button(this, KeyID.B),
            X = new Button(this, KeyID.X),
            Y = new Button(this, KeyID.Y),
            DpadUp = new Button(this, KeyID.DpadUp),
            DpadDown = new Button(this, KeyID.DpadDown),
            DpadLeft = new Button(this, KeyID.DpadLeft),
            DpadRight = new Button(this, KeyID.DpadRight),
            LeftStickButton = new Button(this, KeyID.LeftStickButton),
            RightStickButton = new Button(this, KeyID.RightStickButton),
            Start = new Button(this, KeyID.Start),
            Back = new Button(this, KeyID.Back),
            Guide = new Button(this, KeyID.Guide),
            LeftBumper = new Button(this, KeyID.LeftBumper),
            RightBumper = new Button(this, KeyID.RightBumper);
    public final Joystick
            LeftJoystick = new Joystick(this, KeyID.LeftJoystick),
            RightJoystick = new Joystick(this, KeyID.RightJoystick);
    public final Trigger
            LeftTrigger = new Trigger(this, KeyID.LeftTrigger),
            RightTrigger = new Trigger(this, KeyID.RightTrigger);
    private final Key[] keys = {A, B, X, Y, DpadUp, DpadDown, DpadLeft, DpadRight, LeftStickButton, RightStickButton, Start, Back, Guide, LeftBumper, RightBumper, LeftJoystick, RightJoystick, LeftTrigger, RightTrigger};

    public GamepadEx(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public void loop(){
        for(Key key : keys){
            key.updateState();
        }
    }

    boolean getButton(KeyID key){
        boolean value = false;
        switch (key){
            case A:
                value = gamepad.a;
                break;
            case B:
                value = gamepad.b;
                break;
            case X:
                value = gamepad.x;
                break;
            case Y:
                value = gamepad.y;
                break;
            case DpadUp:
                value = gamepad.dpad_up;
                break;
            case DpadDown:
                value = gamepad.dpad_down;
                break;
            case DpadLeft:
                value = gamepad.dpad_left;
                break;
            case DpadRight:
                value = gamepad.dpad_right;
                break;
            case LeftStickButton:
                value = gamepad.left_stick_button;
                break;
            case RightStickButton:
                value = gamepad.right_stick_button;
                break;
            case Start:
                value = gamepad.start;
                break;
            case Back:
                value = gamepad.back;
                break;
            case Guide:
                value = gamepad.guide;
                break;
            case LeftBumper:
                value = gamepad.left_bumper;
                break;
            case RightBumper:
                value = gamepad.right_bumper;
                break;
        }
        return value;
    }

    double getTrigger(KeyID key){
        double value = 0;
        switch (key){
            case LeftTrigger:
                value = gamepad.left_trigger;
                break;
            case RightTrigger:
                value = gamepad.right_trigger;
                break;
        }
        return value;
    }

    Point getJoystick(KeyID key){
        Point value = new Point();
        switch (key){
            case LeftJoystick:
                value = new Point(gamepad.left_stick_x, -gamepad.left_stick_y);
                break;
            case RightJoystick:
                value = new Point(gamepad.right_stick_x, -gamepad.right_stick_y);
                break;
        }
        return value;
    }

}
