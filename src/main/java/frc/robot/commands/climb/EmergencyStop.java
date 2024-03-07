package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class EmergencyStop extends StormCommand {
    Climber mClimber;

    public EmergencyStop(Climber c){
        mClimber = c;

        addRequirements(c);
    }

    @Override
    public void initialize() {


    }
    public void execute(){
        mClimber.stop();

    }
    public boolean isFinished(){
        return(mClimber.isIdle());
    }
}
