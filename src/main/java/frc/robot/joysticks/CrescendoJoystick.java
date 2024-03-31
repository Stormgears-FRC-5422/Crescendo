package frc.robot.joysticks;

public abstract class CrescendoJoystick {
    public abstract double getWpiX();

    public abstract double getWpiY();

    public abstract double getOmegaSpeed();

    public abstract boolean getRobotRelative();

    public abstract double getTurbo();

    public abstract boolean shooter();

    public abstract boolean shooterIntake();
    public abstract boolean shooterAmp();

    public abstract boolean intake();

    public abstract boolean zeroGyro();

    public abstract boolean diagnosticShooterIntake();
    public abstract boolean outtake();
    public abstract boolean zeroWheels();

    public abstract boolean climb();
    public abstract boolean eject();
    public abstract boolean home();
    public abstract boolean lower();
    public abstract boolean armPreClimb();

    public abstract boolean climberEmergencyStop();

    public abstract boolean ampPosition();

    public abstract boolean driveNote();

    public abstract void setRumble();

    public abstract void stopRumble();

    public abstract boolean alignApriltag();
}
