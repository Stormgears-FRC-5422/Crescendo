package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.StormCommand;
import frc.robot.subsystems.Shooter;

public class ShooterIntake extends StormCommand {
    private final Shooter s;
    private int counter;

    public ShooterIntake(Shooter s) {
        this.s = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing shooter intake");
        super.initialize();
        s.setShooterState(Shooter.ShooterState.SOURCE_PICKUP_2);
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
    }

    @Override
    public boolean isFinished() {
        if (s.isUpperSensorTriggered()) {
            this.log("upper sensor triggered. isFinished = true");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        s.setShooterState(Shooter.ShooterState.IDLE);
        super.end(interrupted);
    }
}



