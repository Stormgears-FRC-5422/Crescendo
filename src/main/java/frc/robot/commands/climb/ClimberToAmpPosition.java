package frc.robot.commands.climb;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.CrescendoField;
import frc.robot.RobotState;
import frc.robot.subsystems.Climber;

public class ClimberToAmpPosition extends Command {

    Translation2d bottomLeftPoint;
    Translation2d topRightPoint;
    Climber climber = new Climber();
    PidMoveToDegrees pidMoveToDegrees;
    int count = 0;

    RectangularRegionConstraint rectangularRegionConstraint;


    public ClimberToAmpPosition(Climber climber){
            this.climber = climber;
            pidMoveToDegrees = new PidMoveToDegrees(climber, Constants.Climber.ampShootDegrees);
    }

    @Override
    public void initialize() {
        bottomLeftPoint = new Translation2d(0.8475837111473083, 6.9402642250061035);
        topRightPoint = new Translation2d(3.4750959873199463, 8.241388320922852);
        if (RobotState.getInstance().isAllianceRed()) {
            bottomLeftPoint = CrescendoField.remapPose(new Pose2d(bottomLeftPoint, new Rotation2d()), RobotState.StateAlliance.RED).getTranslation();
            topRightPoint = CrescendoField.remapPose(new Pose2d(topRightPoint, new Rotation2d()), RobotState.StateAlliance.RED).getTranslation();
        }

        rectangularRegionConstraint = new RectangularRegionConstraint(bottomLeftPoint, topRightPoint, null);
    }

    @Override
    public void execute() {
        if (rectangularRegionConstraint.isPoseInRegion(RobotState.getInstance().getPose())
            && RobotState.getInstance().getPose().getRotation().getRadians() > 1.2
        && RobotState.getInstance().getVisionPose().getRotation().getRadians() < 3.1
        && count == 0) {
                pidMoveToDegrees.schedule();
                count = 1;
        } else {
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
