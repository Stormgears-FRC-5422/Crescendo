package frc.robot.subsystems.drive;

import java.io.IOException;

public class DrivetrainFactory {
    protected static DrivetrainBase instance;
    public static boolean driveFlip = true;

    public static DrivetrainBase getInstance(String driveType) throws IllegalDriveTypeException {
        if (instance == null) {
            System.out.println("Initializing " + driveType);
            switch (driveType.toLowerCase()) {
                case "swervedrive" -> instance = new SwerveDriveTrain();
                case "swervediagnosticdrive" -> instance = new SwerveDiagnosticDriveTrain();
                case "yagsldrive" -> {
                    try {
                        instance = new YagslDriveTrain();
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                }
                case "mecanumdrive" -> instance = new MecanumDrivetrain();
                case "ctrdrive" -> {
                    instance = new CTRDrivetrain();
                    driveFlip = false;
                }
                case "krishdrive" -> instance = new KrishDrive();
                default -> throw new IllegalDriveTypeException("Illegal Drive Type: " + driveType + " ---!");
            }
        }
        return instance;
    }
}
