package frc.utils.joysticks;


public interface DriveJoystick {
    double getTriggerSpeed();

    double getRightTrigger();

    double getLeftTrigger();

    double getWpiXSpeed();
    double getWpiYSpeed();
    double getOmegaSpeed();
}
