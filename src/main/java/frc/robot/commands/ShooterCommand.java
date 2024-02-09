package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.joysticks.StormXboxController;

public class ShooterCommand extends Command {

    double Setpoint = 0.1;
    double error;
    double kp = 1;   //subject to change later because we have to experiment to see what works best
    double newSpeed;
    private final ShooterSubsystem shooter;
    private CrescendoJoystick joystick;

    public ShooterCommand(ShooterSubsystem shooter, CrescendoJoystick joystick) {
        this.shooter = shooter;
        this.joystick = joystick;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command runnin");
    }

    @Override
    public void execute() {
        if (joystick.shooter()) {
            shooter.setShooterSpeed(1);
        } else if (joystick.shooterIntake()) {
            shooter.setShooterSpeed(-0.2);
        } else if (joystick.shooterAmp()) {
            shooter.setShooterSpeed(0.2);
        } else {
            shooter.setShooterSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
