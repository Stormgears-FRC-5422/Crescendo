package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {}
    @Override
    public void periodic() {
        NetworkTable tableInstance = NetworkTableInstance.getDefault().getTable("limelight");
    }




}
