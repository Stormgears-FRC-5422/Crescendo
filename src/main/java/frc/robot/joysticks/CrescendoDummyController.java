package frc.robot.joysticks;

import frc.utils.joysticks.StormXboxController;

public class CrescendoDummyController extends CrescendoJoystick {
    CrescendoDummyController(int port) {

    }

    public double getWpiX() {
        return 0;
    }

    public double getWpiY() {
        return 0;
    }

    public double getOmegaSpeed() {
        return 0;
    }

    public boolean getRobotRelative() { return false; }

    public boolean getTurbo() { return false; }

    @Override
    public boolean shooter() {
        return false;
    }

    @Override
    public boolean shooterIntake() {
        return false;
    }

    @Override
    public boolean shooterAmp() {
        return false;
    }

    @Override
    public boolean intake() {
        return false;
    }

    public boolean zeroGyro() {
        return false;
    }
}
