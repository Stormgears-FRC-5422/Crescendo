package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable tableInstance;
    double ty;
    double mountAngle = 25.0;
    double limelightHeight = 20.0;
    double goalHeight = 60.0;
    public VisionSubsystem() {}

    public double getDistance(){
        ty = tableInstance.getEntry("ty").getDouble(0.0);
        double degrees = mountAngle + ty;
        double radians = degrees * (3.14159 / 180.0);
        double distance = (goalHeight - limelightHeight) / Math.tan(radians);
        return distance;
    }
    public double getAngleDifferential(){
        return ty;
    }
    @Override
    public void periodic() {
        tableInstance = NetworkTableInstance.getDefault().getTable("limelight");
    }




}
