package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleboardConstants {

    public ShuffleboardTab drivetrainTab, preRoundTab;
    public ShuffleboardLayout autoSelectionLayout;
    private static ShuffleboardConstants instance;

    public static ShuffleboardConstants getInstance() {
        if (instance != null) return instance;
        instance = new ShuffleboardConstants();
        return instance;
    }

    private ShuffleboardConstants() {
        drivetrainTab = Shuffleboard.getTab("Drivetrain");
        preRoundTab = Shuffleboard.getTab("Pre Round");

        autoSelectionLayout = preRoundTab
            .getLayout("Auto Selector", BuiltInLayouts.kList)
            .withPosition(0, 0).withSize(2, 5);
    }

}
