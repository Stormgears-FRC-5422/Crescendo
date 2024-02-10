package frc.robot.subsystems.drive;

public class IllegalDriveTypeException extends Exception {
    public IllegalDriveTypeException(String message) {
        super(message);
    }
}