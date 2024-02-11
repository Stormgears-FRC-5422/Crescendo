package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class CrescendoField {

    // By definition, all coordinates should be stored in Blue Alliance coordinates.
    // The far right corner on the Blue alliance side is the origin.
    // coordinates are in meters unless otherwise indicated.
    public final static double FIELD_LENGTH = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
    public final static double FIELD_WIDTH = Units.feetToMeters(26) + Units.inchesToMeters(3.5);

    public static Pose2d remapPose(Pose2d pose2d, boolean blueAlliance) {
        if (blueAlliance)
            return pose2d;
        else
            return mirrorPose(pose2d);
    }

    private static double mirrorX(double x) {
        return FIELD_LENGTH - x;
    }
    private static double mirrorY(double y) {return y;}

    private static Translation2d mirrorTranslation(Translation2d translation) {
        return new Translation2d(mirrorX(translation.getX()), translation.getY());
    }

    private static Rotation2d mirrorRotation(Rotation2d rotation) {
        return new Rotation2d(-rotation.getCos(), rotation.getSin());
    }

    private static Pose2d mirrorPose(Pose2d pose) {
        Translation2d translationMirrored = mirrorTranslation(pose.getTranslation());
        Rotation2d rotationMirrored = mirrorRotation(pose.getRotation());
        return new Pose2d(translationMirrored, rotationMirrored);
    }
}
