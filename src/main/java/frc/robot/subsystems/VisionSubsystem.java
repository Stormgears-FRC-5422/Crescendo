package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.utils.vision.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private boolean initialized = false;
    private NetworkTableEntry tTarget = null;
    private NetworkTableEntry txEntry = null;
    private NetworkTableEntry tyEntry = null;
    private NetworkTableEntry taEntry = null;
    private NetworkTableEntry botpose = null;
    private NetworkTableEntry targetpose = null;
    private NetworkTableEntry tl = null;
    private NetworkTableEntry cl = null;
    private Alliance alliance = Alliance.Blue;
    NetworkTable tableInstance;

    double ty;

    double tx;
    double MOUNT_ANGLE_DEGREES = 25.0;
    double LIMELIGHT_HEIGHT = 20.0;
    double GOAL_HEIGHT = 60.0;

    public VisionSubsystem() {

        initialized = true;
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

    public boolean isInitialized() {
        return this.initialized;
    }

    public boolean hasTargets() {
        boolean hits = false;
        //SmartDashboard.putBoolean("isInitialized", isInitialized());
        if (isInitialized()) {
            hits = (tableInstance.getEntry("tv").getDouble(0.0) == 1.0);
        }
        return hits;
    }

    public double[] botPose() {
        double[] botPose = null;
        //SmartDashboard.putBoolean("Limelight Inititialized", isInitialized());
        if (isInitialized()) {
            botPose = botpose.getDoubleArray(new double[7]);
        }
        return botPose;
    }

    public double tl() {
        double tL = 0.0;
        if (isInitialized()) {
            tL = tl.getDouble(0.0);
        }
        return tL;
    }

    public double cl() {
        double cL = 0.0;
        if (isInitialized()) {
            cL = cl.getDouble(0.0);
        }
        return cL;
    }

    public double targetDist() {
        double[] targetPose = null;
        if (isInitialized()) {
            targetPose = targetpose.getDoubleArray(new double[3]);
        }
        Translation3d dist = new Translation3d(targetPose[0], targetPose[1], targetPose[2]);
        return dist.getDistance(new Translation3d());
    }


    public double targetArea() {
        double dArea = 0.0;
        if (isInitialized()) {
            dArea = taEntry.getDouble(0.0);
        }
        return dArea;
    }

    public double tv() {
        double tv = 0.0;
        if (isInitialized()) {
            tv = tTarget.getDouble(0.0);
        }
        return tv;
    }


    @Override
    public void periodic() {
        tableInstance = NetworkTableInstance.getDefault().getTable("limelight");

        if (this.alliance == Alliance.Blue) {
            botpose = tableInstance.getEntry("botpose_wpiblue");
        } else {
            botpose = tableInstance.getEntry("botpose_wpired");
        }

        tTarget = tableInstance.getEntry("tv");
        txEntry = tableInstance.getEntry("tx");
        tyEntry = tableInstance.getEntry("ty");
        taEntry = tableInstance.getEntry("ta");

        targetpose = tableInstance.getEntry("targetpose_robotspace");
        tl = tableInstance.getEntry("tl");
        cl = tableInstance.getEntry("cl");

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);

        SmartDashboard.putNumber("Vision Pose Rotation",
            LimelightHelpers.toPose2D(botpose.getDoubleArray(new double[7])).getRotation().getDegrees());
        SmartDashboard.putNumber("Vision Pose X",
            LimelightHelpers.toPose2D(botpose.getDoubleArray(new double[7])).getX());
        SmartDashboard.putNumber("Vision Pose Y",
            LimelightHelpers.toPose2D(botpose.getDoubleArray(new double[7])).getY());

        RobotState.getInstance().setVisionPose(LimelightHelpers.toPose2D(botpose.getDoubleArray(new double[6])));

//        System.out.println(LimelightHelpers.toPose2D(botpose.getDoubleArray(new double[6])));


        double[] pose = tableInstance.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);


    }


}
