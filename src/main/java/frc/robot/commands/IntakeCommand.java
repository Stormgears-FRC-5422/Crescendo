package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.subsystems.IntakeSubSystem;

public class IntakeCommand extends Command {

    double Setpoint = 0.2;
    double error;
    double kp = 1;   //subject to change later because we have to experiment to see what works best
    double newSpeed;
    private final IntakeSubSystem intake;
    private CrescendoJoystick joystick;

    public IntakeCommand(IntakeSubSystem intake, CrescendoJoystick joystick) {
        this.intake = intake;
        this.joystick = joystick;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter command runnin");

    }

    @Override
    public void execute() {
        if (joystick.intake()) {

//            error = Setpoint - (intake.getIntakeSpeed());
//            newSpeed = Setpoint + error * kp;
            intake.setIntakeSpeed(0.2);

        } else {
            intake.setIntakeSpeed(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
