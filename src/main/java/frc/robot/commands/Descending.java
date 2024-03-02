package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class Descending extends StormCommand{
    Climber mClimber;
    
    public Descending(Climber c){
        mClimber = c;
    }

    @Override
    public void initialize() {

        
    }
    public void execute(){
        if(!mClimber.isTooFarBack()){
            mClimber.Descend();
        }
        else{
            mClimber.Stop();
        }
    }


    






    
    
}