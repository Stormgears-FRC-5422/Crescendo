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

    public double getTurbo() {
        return controller.getRightTrigger();
    }

    @Override
    public boolean shooter() {
        return controller.getBButtonIsHeld();
    }

    @Override
    public boolean shooterIntake() {
        return controller.getYButtonIsHeld();
    }

    @Override
    public boolean shooterAmp() {
        return controller.getXButtonIsHeld();
    }

    @Override
    public boolean intake() {
        return controller.getAButtonIsHeld();
    }

    public boolean zeroGyro() {
//        return controller.getRightBumperIsPressed() && controller.getBackIsPressed();
        return controller.getRightBumperIsPressed();
    }

    @Override
    public boolean diagnosticShooterIntake() {
        return controller.getDownArrowPressed();
    }

    @Override
    public boolean outtake() {
        return controller.getUpArrowPressed();
    }

    @Override
    public boolean zeroWheels() {
        return   controller.getLeftBumperIsPressed();
    }

    public boolean home() {
        return controller.getLeftArrowPressed();
    }

    public boolean climb() {
        return controller.getRightArrowPressed();
    }

    public boolean climberEmergencyStop() {
        return controller.getLeftLittleButtonIsHeld();
    }

    @Override
    public boolean ampPosition() {
        return false;
    }


}
