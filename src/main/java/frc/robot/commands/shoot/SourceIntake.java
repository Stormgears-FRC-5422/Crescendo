package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Shooter;

public class SourceIntake extends StormCommand {
    private final Shooter s;
    private int step = 0;  // To keep track of where we are in the command

    public SourceIntake(Shooter s) {
        this.s = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        super.initialize();
        step = 0;
    }

    @Override
    public void execute() {
        switch (step) {
            case 0: // At the top of the shooter. Begin to run in reverse
                s.setShooterState(Shooter.ShooterState.SOURCE_PICKUP_1);
                step = 1;
                break;
            case 1: // Wait for sensor to trigger
                if (s.isUpperSensorTriggered()) {
                    s.setShooterState(Shooter.ShooterState.SOURCE_PICKUP_2);
                    step = 2;
                }
                break;
            case 2: // Wait for sensor to clear
                if (!s.isUpperSensorTriggered()) {
                    s.setShooterState(Shooter.ShooterState.GROUND_PICKUP);
                    step = 3;
                }
                break;
            case 3: // Move slightly forward, until staged for shooting as in ground pickup
                // pass - nothing else to do until isFinished
                break;
            default:
                step = -1;
                this.log("Illegal state in SourceIntake Command!");
        }
    }

    @Override
    public boolean isFinished() {
        return (step == 3 && s.isUpperSensorTriggered()) || step == -1;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || step == -1) {
            s.setShooterState(Shooter.ShooterState.BAD);
        } else {
            s.setShooterState(Shooter.ShooterState.STAGED_FOR_SHOOTING);
        }

        super.end(interrupted);
    }
}



