package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable tableInstance;

    double ty;

    double tx;
    double MOUNT_ANGLE_DEGREES = 25.0;
    double LIMELIGHT_HEIGHT = 20.0;
    double GOAL_HEIGHT = 60.0;

    public VisionSubsystem() {
    }

    public double getDistance() {
        ty = tableInstance.getEntry("ty").getDouble(0.0);
        tx = tableInstance.getEntry("tx").getDouble(0.0);
        double degrees = MOUNT_ANGLE_DEGREES + ty;
        double radians = Math.toRadians(degrees);
        return ((GOAL_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(radians)) / Math.cos(tx);
    }

    public double getVerticalAngle() {
        return ty;
    }

    public double getHorizontalAngle() {
        return tx;
    }

    @Override
    public void periodic() {
        double[] pose = tableInstance.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);


        tableInstance = NetworkTableInstance.getDefault().getTable("limelight");
    }
}
