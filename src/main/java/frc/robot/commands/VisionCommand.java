package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends Command {
    VisionSubsystem visionSubsystem;
    double distanceFromTarget;

    VisionCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);

    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        distanceFromTarget = visionSubsystem.getDistance();
        SmartDashboard.putNumber("Distance From Target", distanceFromTarget);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    public void align(){
        double horizontalAngle = visionSubsystem.getHorizontalAngle();
        double verticalAngle = visionSubsystem.getVerticalAngle();


        
    }

}
