package frc.robot.joysticks;

import frc.utils.joysticks.StormLogitechController;

public class CrescendoLogitechController extends CrescendoJoystick {
    StormLogitechController controller;

    CrescendoLogitechController(int port) {
        controller = new StormLogitechController(port);
    }

    @Override
    public boolean eject() {
        return false;
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
//        return controller.getRawButton(11);
        return false;

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
//        return false;

    }

    @Override
    public boolean shooterAmp() {
        return controller.getRawButton(4);
//        return false;
    }

    @Override
    public boolean intake() {
        return controller.getRawButton(2);
//        return false;
    }

    public boolean zeroGyro() {
        return false;
//        return controller.getRawButton(5);
    }

    @Override
    public boolean diagnosticShooterIntake() {
//        return controller.getRawButton(6);
        return false;

    }

    @Override
    public boolean outtake() {
        return false;
    }

    @Override
    public boolean climb() {
//        return controller.getRawButton(9);
        return false;
    }

    @Override
    public boolean home() {
//        return controller.getRawButton(10);
        return false;

    }

    public boolean lower() {
        return false;
    }
    public boolean armPreClimb() {
        return false;
    }

    @Override
    public boolean climberEmergencyStop() {

//        return controller.getRawButton(7);
        return false;

    }

    @Override
    public boolean ampPosition() {
        return false;
    }

    public void setRumble() {

    }

    @Override
    public boolean driveNote() {
        return false;
    }

    @Override
    public void stopRumble() {

    }

    @Override
    public boolean alignApriltag() {
        return false;
    }


}
