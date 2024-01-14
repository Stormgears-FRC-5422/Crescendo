package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.utils.joysticks.StormXboxController;

public class ShooterCommand extends Command {
    private final Shooter shooter;
    private StormXboxController stormXboxController;
    public ShooterCommand(Shooter shooter, StormXboxController stormXboxController){
        this.shooter = shooter;
        this.stormXboxController = stormXboxController;
    }

}
