package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class Climbing extends StormCommand{
    Climber mClimber;
    
    public Climbing(Climber c){
        mClimber = c;
    }

    @Override
    public void initialize() {

        
    }
    public void execute(){
        if(!mClimber.isTooFarForward()){
            mClimber.Climb();
        }
        else{
            mClimber.Stop();
        }
    }


    






    
    
}
