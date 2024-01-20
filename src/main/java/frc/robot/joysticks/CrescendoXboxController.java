package frc.robot.joysticks;

import frc.utils.joysticks.StormXboxController;

public class CrescendoXboxController extends CrescendoJoystick {
    StormXboxController controller;

    CrescendoXboxController(int port) {
        controller = new StormXboxController(port);
    }

    public double getWpiX() {
        return controller.getWpiXSpeed();
    }

    public double getWpiY() {
        return controller.getWpiYSpeed();
    }

    public double getOmegaSpeed() {
        return controller.getOmegaSpeed();
    }

    public boolean getRobotRelative() {
        return controller.getLeftTrigger() > 0.2;
    }

    public boolean getTurbo() {
        return controller.getRightTrigger() > 0.2;
    }
}
