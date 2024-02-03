package frc.robot.joysticks;

public abstract class CrescendoJoystick {
    public abstract double getWpiX();

    public abstract double getWpiY();

    public abstract double getOmegaSpeed();

    public abstract boolean getRobotRelative();

    public abstract boolean getTurbo();

    public abstract boolean shooter();

    public abstract boolean shooterIntake();
    public abstract boolean shooterAmp();

    public abstract boolean intake();
}
