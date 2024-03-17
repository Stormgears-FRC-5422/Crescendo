package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.StormCommand;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.joysticks.CrescendoXboxController;
import frc.robot.subsystems.Shooter;

public class GroundPickup extends StormCommand {
    private final Shooter s;


    public GroundPickup(Shooter s) {
        this.s = s;

        addRequirements(s);
    }

    @Override
    public void initialize() {
        System.out.println("INIT GROUND PICKUP!!!");
        super.initialize();
        s.setShooterState(Shooter.ShooterState.GROUND_PICKUP);
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

        s.setShooterState(Shooter.ShooterState.STAGED_FOR_SHOOTING);
        super.end(interrupted);



    }

}



