package frc.robot.commands.climb;

import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class Home extends StormCommand {
    Climber climber;

    public Home(Climber c) {
        climber = c;

        addRequirements(c);
    }

    @Override
    public void initialize() {
        super.initialize();
        climber.setClimberState(Climber.ClimberState.HOMING);
    }

    public void execute() {
    }

    public boolean isFinished() {
        if (climber.isHome()) {
            this.log("Climber is homed. isFinished = true");

            return true;
        }
        return false;
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            climber.setClimberState(Climber.ClimberState.IDLE);
        } else {
            climber.setClimberState(Climber.ClimberState.HOME);
        }

        super.end(interrupted);
    }


}
