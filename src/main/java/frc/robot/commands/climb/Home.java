package frc.robot.commands.climb;

import frc.robot.Constants;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Climber;

public class Home extends StormCommand {
    Climber climber;
    boolean atHome;

    public Home(Climber c) {
        climber = c;
        addRequirements(c);
    }

    @Override
    public void initialize() {
        super.initialize();
        atHome = false;

        if (Constants.Climber.useCurrentLimitHomeStrategy) {
            climber.setClimberState(Climber.ClimberState.CURRENT_HOMING);
        } else {
            climber.setClimberState(Climber.ClimberState.SWITCH_HOMING);
        }
    }

    public void execute() {
        if (Constants.Climber.useCurrentLimitHomeStrategy) {
            if (climber.isAtStallLimit() && !atHome) {
                climber.setClimberState(Climber.ClimberState.HOME);
                atHome = true;
            }
        } else {
            if (climber.isHome() & !atHome) {
                climber.setClimberState(Climber.ClimberState.HOME);
                atHome = true;
            }
        }
     }

    public boolean isFinished() {
//        System.out.println("climber is " + (climber.isHome()? "home" : "not home"));
//        System.out.println("climber is " + (climber.isStopped()? "stopped" : "not stopped"));
        if (climber.isHome() && climber.hasStopped()) {
            this.log("Climber is homed. isFinished = true");
            return true;
        }
        return false;
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            climber.setClimberState(Climber.ClimberState.IDLE_BRAKE);
        } else {
            climber.setClimberState(Climber.ClimberState.IDLE_BRAKE);
        }
        super.end(interrupted);
    }


}
