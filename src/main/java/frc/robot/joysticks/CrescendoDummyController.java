package frc.robot.joysticks;

public class CrescendoDummyController extends CrescendoJoystick {
    CrescendoDummyController(int port) {

    }

    @Override
    public boolean eject() {
        return false;
    }

    @Override
    public boolean zeroWheels() {
        return false;
    }

    @Override
    public boolean climb() {
        return false;
    }

    @Override
    public boolean home() {
        return false;
    }

    @Override
    public boolean armPreClimb() { return false; }

    @Override
    public boolean lower() {
        return false;
    }

    @Override
    public boolean climberEmergencyStop() {
        return false;
    }

    @Override
    public boolean ampPosition() {
        return false;
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

    public double getTurbo() { return 0; }

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

    @Override
    public boolean diagnosticShooterIntake() {
        return false;
    }

    @Override
    public boolean outtake() {
        return false;
    }

    @Override
    public boolean driveNote() {
        return false;
    }

    public void setRumble() {

    }

    @Override
    public void stopRumble() {

    }

    @Override
    public boolean alignApriltag() {
        return false;
    }
}
