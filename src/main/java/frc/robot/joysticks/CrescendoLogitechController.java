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

    public double getTurbo() {
        return 1;
    }

    @Override
    public boolean zeroWheels() {
        return false;
    }

    @Override
    public boolean shooter() {
        return controller.getRawButton(1);
    }

    @Override
    public boolean shooterIntake() {
        return controller.getRawButton(3);
    }

    @Override
    public boolean shooterAmp() {
        return controller.getRawButton(4);
    }

    @Override
    public boolean intake() {
        return controller.getRawButton(2);
    }

    public boolean zeroGyro() {
        return controller.getRawButton(5);
    }

    @Override
    public boolean diagnosticShooterIntake() {
        return controller.getRawButton(6);
    }

    @Override
    public boolean outtake() {
        return false;
    }

    @Override
    public boolean climb() {
        
        return controller.getRawButton(9);
    }

    @Override
    public boolean home() {
        
        return controller.getRawButton(10);
    }

    @Override
    public boolean climberEmergencyStop() {
        
        return controller.getRawButton(7);
    }
    
}
