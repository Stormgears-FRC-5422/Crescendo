package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SourceIntake extends Command {
    private final Shooter s;
    private int commandState = 0;  // To keep track of where we are in the command

    public SourceIntake(Shooter s) {
        this.s = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command running");
        commandState = 0;
    }

    @Override
    public void execute() {
        switch (commandState) {
            case 0: // At the top of the shooter. Begin to run in reverse
                s.ShooterStateMachine(Shooter.ShooterStates.SOURCE_PICKUP_1);
                commandState = 1;
                break;
            case 1: // Wait for sensor to trigger
                if (s.isUpperSensorTriggered()) {
                    s.ShooterStateMachine(Shooter.ShooterStates.SOURCE_PICKUP_2);
                    commandState = 2;
                }
                break;
            case 2: // Wait for sensor to clear
                if (!s.isUpperSensorTriggered()) {
                    s.ShooterStateMachine(Shooter.ShooterStates.GROUND_PICKUP);
                    commandState = 3;
                }
                break;
            case 3: // Move slightly forward, until staged for shooting as in ground pickup
                // pass - nothing else to do until isFinished
                break;
            default:
                commandState = -1;
                System.out.println("Illegal state in SourceIntake Command!");
        }
    }

    @Override
    public boolean isFinished() {
        return (commandState == 3 && s.isUpperSensorTriggered()) || commandState == -1;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || commandState == -1) {
            s.ShooterStateMachine(Shooter.ShooterStates.BAD);
        } else {
            s.ShooterStateMachine(Shooter.ShooterStates.STAGED_FOR_SHOOTING);
        }
    }
}



