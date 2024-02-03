package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.utils.joysticks.StormXboxController;

public class IntakeCommand extends Command {
    
    double Setpoint = 0.2;
    double error; 
    double kp=1;   //subject to change later because we have to experiment to see what works best
    double newSpeed;
    private final Intake intake;
    private StormXboxController stormXboxController;

    public ShooterCommand(Intake intake, StormXboxController stormXboxController) {
        this.intake = intake;
        this.stormXboxController = stormXboxController;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command runnin");

    }

    @Override
    public void execute() {
        if (stormXboxController.getBButtonIsHeld() ){
            
            error = Setpoint-(intake.getShooterSpeed());
            newSpeed = Setpoint+error*kp;
            intake.setIntakeSpeed(newSpeed);
                      
        }
        else {
            intake.setIntakeSpeed(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}