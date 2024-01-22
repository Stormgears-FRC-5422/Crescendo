package frc.robot.subsystems.drive;

import java.io.IOException;

public class DrivetrainFactory {
    protected static DrivetrainBase instance;

    public static DrivetrainBase getInstance(String driveType) throws IllegalDriveTypeException {
        if (instance ==  null) {
            System.out.printf("Initializing %s", driveType);
            switch (driveType) {
                case "SwerveDrive" -> instance = new SwerveDriveTrain();
                case "SwerveDiagnosticDrive" -> instance = new SwerveDiagnosticDriveTrain();
                case "YagslDrive" -> {
                    try {
                        instance = new YagslDriveTrain();
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                }
                case "MecanumDrive" -> instance = new MecanumDrivetrain();
                default -> throw new IllegalDriveTypeException("Illegal Drive Type: " + driveType + " ---!");
            }
        }
        return instance;
    }
}