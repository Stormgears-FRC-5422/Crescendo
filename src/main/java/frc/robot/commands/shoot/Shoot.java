package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.StormCommand;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.subsystems.Shooter;

public class Shoot extends StormCommand {
    private final Shooter s;
    private int counter;

    public Shoot(Shooter s) {
        this.s = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        super.initialize();
        s.setShooterState(Shooter.ShooterState.SPEAKER_SHOOTING);
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
    }

    @Override
    public boolean isFinished() {
        return counter > Constants.Shooter.shootForIterations;
    }

    @Override
    public void end(boolean interrupted) {
        s.setShooterState(Shooter.ShooterState.IDLE);
        super.end(interrupted);
    }
}



