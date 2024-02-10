package frc.utils.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.ButtonBoard;

public class StormLogitechController extends Joystick implements DriveJoystick {

    public static final int XAxis = 0;
    public static final int YAxis = 1;
    public static final int ZAxis = 2;
    public static final int sliderAxis = 3;

    public StormLogitechController(int port) {
        super(port);
    }

    private double applyNullZone(double value) {
//        if (Math.abs(value) < kStickNullSize)
//            return 0;
//
//        return ((value - Math.signum(value) * kStickNullSize) / (1.0 - kStickNullSize));
        return MathUtil.applyDeadband(value, ButtonBoard.stickNullSize);
    }

    public double getXAxis() {
        return applyNullZone(getRawAxis(XAxis));
    }
    public double getYAxis() {
        return -applyNullZone(getRawAxis(YAxis));
    }
    public double getZAxis() {
        return applyNullZone(getRawAxis(ZAxis));
    }
    public double getSliderAxis() {
        return (-applyNullZone(getRawAxis(sliderAxis)) + 1) / 2;
    }

    public double getWpiXAxis(){
        return getYAxis();
    }
    public double getWpiYAxis(){
        return -getXAxis();
    }
    public double getWpiZAxis(){
        return -getZAxis();
    }

    public double getWPIPOVAngle() {
        int povAngle = getPOV();
//        System.out.println("POV angle: " + povAngle);
        if (povAngle == -1) return -1;
        if (povAngle == 0) return 0;
        double inverse = 360.0 - povAngle;
        if (inverse > 180) return  (inverse % 180) - 180.0;
        return inverse;
    }

    @Override
    public double getTriggerSpeed() {
        return 0;
    }

    @Override
    public double getRightTrigger() {
        return 0;
    }

    @Override
    public double getLeftTrigger() {
        return 0;
    }

    @Override
    public double getWpiXSpeed() {
        return getWpiXAxis();
    }

    @Override
    public double getWpiYSpeed() {
        return getWpiYAxis();
    }

    @Override
    public double getOmegaSpeed() {
        return getWpiZAxis();
    }
}

