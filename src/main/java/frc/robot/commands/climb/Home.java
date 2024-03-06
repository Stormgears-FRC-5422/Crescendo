package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class Home extends StormCommand {
    Climber m_climber;

    public Home(Climber c){
        m_climber = c;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_climber.setClimberState(Climber.ClimberState.HOMING);
    }

    public void execute(){
    }

    public boolean isFinished() {
//        if (m_climber.isHome()) {
//            this.log("Climber is homed. isFinished = true");
//
//            return true;
//        }
//
//        return false;
        return true;
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            m_climber.setClimberState(Climber.ClimberState.IDLE);
        } else {
            m_climber.setClimberState(Climber.ClimberState.HOME);
        }

        super.end(interrupted);
    }












}
