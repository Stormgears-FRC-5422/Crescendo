package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class EmergencyStop extends StormCommand{
    Climber mClimber;
    
    public EmergencyStop(Climber c){
        mClimber = c;
    }

    @Override
    public void initialize() {

        
    }
    public void execute(){
        mClimber.Stop();
    }
}