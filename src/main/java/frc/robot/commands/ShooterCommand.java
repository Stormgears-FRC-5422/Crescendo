package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.utils.joysticks.StormXboxController;

public class ShooterCommand extends Command {
    
    double Setpoint = 0.2;
    double error; 
    double kp=1;   //subject to change later because we have to experiment to see what works best
    double newSpeed;
    private final Shooter shooter;
    private StormXboxController stormXboxController;

    public ShooterCommand(Shooter shooter, StormXboxController stormXboxController) {
        this.shooter = shooter;
        this.stormXboxController = stormXboxController;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command runnin");

    }

    @Override
    public void execute() {
        if (stormXboxController.getBButtonIsHeld() ){
            
            error = Setpoint-(shooter.getShooterSpeed());
            newSpeed = Setpoint+error*kp;
            shooter.setShooterSpeed(newSpeed);
                      
        }
        else {
            shooter.setShooterSpeed(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}


