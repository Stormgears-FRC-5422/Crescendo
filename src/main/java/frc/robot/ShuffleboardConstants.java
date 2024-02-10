package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardConstants {

    public ShuffleboardTab drivetrainTab;
    private static ShuffleboardConstants instance;

    public static ShuffleboardConstants getInstance() {
        if (instance != null) return instance;
        instance = new ShuffleboardConstants();
        return instance;
    }

    private ShuffleboardConstants() {
        drivetrainTab = Shuffleboard.getTab("Drivetrain");
    }

}
