package frc.robot.joysticks;

import edu.wpi.first.wpilibj.GenericHID;
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
//        return true;
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

        //return controller.getRightLittleButtonIsHeld();
        return false;
    }

    @Override
    public boolean zeroWheels() {
//        return   controller.getLeftBumperIsPressed();
        return false;
    }

    public boolean home() {
        return controller.getLeftArrowPressed();
    }

    public boolean lower() { return controller.getRightArrowPressed(); }

    public boolean armPreClimb() { return controller.getUpArrowPressed(); }

    public boolean climb() {
        return controller.getDownArrowPressed();
    }

    public boolean eject() {
        return controller.getLeftLittleButtonIsHeld();
    }

    @Override
    public boolean climberEmergencyStop() {
        return false;
    }

    @Override
    public boolean ampPosition() {

        return controller.getLeftBumperIsPressed();
//            return false;
    }

    @Override
    public boolean driveNote() {
        return controller.getLeftBumperIsHeld();
    }

    public void setRumble() {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
    }

    @Override
    public void stopRumble() {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    public boolean alignApriltag(){
        return controller.getRightLittleButtonIsHeld();
    }

}
