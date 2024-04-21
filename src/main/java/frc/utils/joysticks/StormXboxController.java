package frc.utils.joysticks;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.ButtonBoard;

public class StormXboxController extends Joystick implements DriveJoystick {

    // sticks and triggers
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1;
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int rightXAxis = 4;
    public static final int rightYAxis = 5;

    // buttons
    public static final int AButton = 1;
    public static final int BButton = 2;
    public static final int XButton = 3;
    public static final int YButton = 4;
    public static final int leftBumper = 5;
    public static final int rightBumper = 6;
    public static final int littleLeftButton = 7;
    public static final int littleRightButton = 8;
    public static final int stickLeftButton = 9;
    public static final int stickRightButton = 10;

    public StormXboxController(int port) {
        super(port);
    }

    protected double applyNullZone(double value) {
//        if (abs(value) < kStickNullSize) return 0;
//
//        // Scale up from 0 rather than jumping to the input value
//        return ( (value - signum(value) * kStickNullSize) / (1.0 - kStickNullSize) );
        return MathUtil.applyDeadband(value, ButtonBoard.stickNullSize);
    }

    @Override
    public double getRightTrigger() {
        return getRawAxis(rightTrigger);
    }

    @Override
    public double getLeftTrigger() {
        return getRawAxis(leftTrigger);
    }

    @Override
    public double getWpiXSpeed() {
        return getLeftJoystickY();
    }

    @Override
    public double getWpiYSpeed() {
        return -getLeftJoystickX();
    }

    @Override
    public double getOmegaSpeed() {
        return -getRightJoystickX();
    }

    @Override
    public double getTriggerSpeed(){ return getRawAxis(rightTrigger) - getRawAxis(leftTrigger); }

    public boolean getAButtonIsHeld() {
        return getRawButton(AButton);
    }

    public boolean getBButtonIsHeld() {
        return getRawButton(BButton);
    }

    public boolean getXButtonIsHeld() {
    return getRawButton(XButton);
    }

    public boolean getYButtonIsHeld() {
    return getRawButton(YButton);
    }

    public boolean getRightBumperIsHeld() {return getRawButton(rightBumper); }

    public boolean getLeftBumperIsHeld() {
        return getRawButton(leftBumper);
    }

    public boolean getRightLittleButtonIsHeld() {
        return getRawButton(littleRightButton);
    }

    public boolean getLeftLittleButtonIsHeld() {
    return getRawButton(littleLeftButton);
    }

    public double getLeftJoystickX() {
        return applyNullZone(getRawAxis(leftXAxis));
    }

    public double getLeftJoystickY() { return applyNullZone(-getRawAxis(leftYAxis)); }

    public double getRightJoystickX() {
        return applyNullZone(getRawAxis(rightXAxis));
    }

    public double getRightJoystickY() {
        return applyNullZone(-getRawAxis(rightYAxis));
    }

    public boolean getBisPressed() {
        return getRawButtonPressed(BButton);
    }

    public boolean getXisPressed(){
        return  getRawButtonPressed(XButton);
    }

    public boolean getBackIsPressed() {
        return getRawButtonPressed(littleLeftButton);
    }

    public boolean getRightBumperIsPressed() {return getRawButtonPressed(rightBumper); }

    public boolean getLeftBumperIsPressed() {
        return getRawButtonPressed(leftBumper);
    }

    public boolean getRotateIsPressed() {
        return getRawButtonPressed(littleRightButton);
    }

    public boolean getAlignedIsPressed(){
        return getRawButtonPressed(stickLeftButton);
    }

    public boolean getStickLeftButton() {
        return getRawButton(stickLeftButton);
    }

    public boolean getStickRightButton() {
        return getRawButton(stickRightButton);
    }

    public boolean getUpArrowPressed() {
        return getPOV() == 0;
    }

    public boolean getDownArrowPressed() {
        return getPOV() == 180;
    }

    public boolean getLeftArrowPressed() {
        return getPOV() == 270;
    }

    public boolean getRightArrowPressed() {
        return getPOV() == 90;
    }

}
