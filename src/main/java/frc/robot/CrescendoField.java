package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class CrescendoField {
    // By definition, all coordinates should be stored in Blue Alliance coordinates.
    // The far right corner on the Blue alliance side is the origin.
    // coordinates are in meters unless otherwise indicated.
    public final static double FIELD_LENGTH = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
    public final static double FIELD_WIDTH = Units.feetToMeters(26) + Units.inchesToMeters(3.5);

    public static Pose2d remapPose(Pose2d pose2d, RobotState.StateAlliance alliance) {
        // We want to make sure that both of these code paths return an entirely new object
        return switch(alliance) {
            case BLUE, MISSING -> new Pose2d(pose2d.getX(), pose2d.getY(), new Rotation2d(pose2d.getRotation().getRadians()));
            case RED -> mirrorPose(pose2d);
        };
    }

    private static double mirrorX(double x) {
        return FIELD_LENGTH - x;
    }
    private static double mirrorY(double y) {return y;}

    private static Translation2d mirrorTranslation(Translation2d translation) {
        return new Translation2d(mirrorX(translation.getX()), mirrorY(translation.getY()));
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
