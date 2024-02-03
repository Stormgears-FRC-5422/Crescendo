package frc.robot.joysticks;

import frc.utils.joysticks.StormLogitechController;

public class CrescendoLogitechController extends CrescendoJoystick {
    StormLogitechController controller;

    CrescendoLogitechController(int port) {
        controller = new StormLogitechController(port);
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
        return controller.getRawButton(11);
    }

    public boolean getTurbo() {
        return controller.getRawButton(2);
    }

}
