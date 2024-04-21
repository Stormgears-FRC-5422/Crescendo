package frc.robot.commands.climb;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import frc.robot.Constants;
import frc.robot.CrescendoField;
import frc.robot.RobotState;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class ClimberToAmpPosition extends StormCommand {
    RobotState state;
    Translation2d bottomLeftPoint;
    Translation2d topRightPoint;
    double rotationMax;
    double rotationMin;
    Climber climber;
    int count = 0;

    RectangularRegionConstraint rectangularRegionConstraint;

    public ClimberToAmpPosition(Climber climber) {
        this.climber = climber;
        state = RobotState.getInstance();
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        super.initialize();

        // These define corners to a bounding rectangle near the amp
        bottomLeftPoint = new Translation2d(0.8475837111473083, 6.9402642250061035);
        topRightPoint = new Translation2d(3.4750959873199463, 8.241388320922852);
        rotationMax = 3.1;
        rotationMin = 1.2;
        if (state.isAllianceRed()) {
            bottomLeftPoint = CrescendoField.remapPose(new Pose2d(bottomLeftPoint, new Rotation2d()), RobotState.StateAlliance.RED).getTranslation();
            topRightPoint = CrescendoField.remapPose(new Pose2d(topRightPoint, new Rotation2d()), RobotState.StateAlliance.RED).getTranslation();
            rotationMax = CrescendoField.remapPose(new Pose2d(0, 0, new Rotation2d(3.1)), RobotState.StateAlliance.RED).getRotation().getRadians();
            rotationMin = CrescendoField.remapPose(new Pose2d(0, 0, new Rotation2d(1.2)), RobotState.StateAlliance.RED).getRotation().getRadians();

        }

        rectangularRegionConstraint = new RectangularRegionConstraint(bottomLeftPoint, topRightPoint, null);
        climber.setClimberState(Climber.ClimberState.MOVE_PID_POSITION);
    }

    @Override
    public void execute() {
        if (rectangularRegionConstraint.isPoseInRegion(state.getPose())
            && state.getPose().getRotation().getRadians() > rotationMin
            && state.getVisionPose().getRotation().getRadians() < rotationMax
            && count == 0) {
            climber.setTargetDegrees(Constants.Climber.ampShootDegrees);
            count = 1;
        } else {
            climber.setTargetDegrees(Constants.Climber.stowDegrees);
            count = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
