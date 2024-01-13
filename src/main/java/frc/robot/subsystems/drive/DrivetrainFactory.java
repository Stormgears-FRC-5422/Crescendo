package frc.robot.subsystems.drive;

public class DrivetrainFactory {
    protected static DrivetrainBase instance;

    public static DrivetrainBase getInstance(String driveType) throws IllegalDriveTypeException {
        if (instance != null)
            return instance;

        switch (driveType) {
            case "SwerveDrive":
                instance = new SwerveDriveTrain();
                return instance;
            case "MecanumDrive":
                instance = new MecanumDrivetrain();
                return instance;
            default:
                throw new IllegalDriveTypeException("Illegal Drive Type: " + driveType + " ---!");
        }
    }
}