package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.joysticks.CrescendoJoystick;

public class RumbleCommand extends SequentialCommandGroup {

    public RumbleCommand(CrescendoJoystick joystick, double time) {
        addCommands(
            new InstantCommand(()-> System.out.println("RUMBLING")),
        new InstantCommand(() -> joystick.setRumble()),
            new WaitCommand(time),
            new InstantCommand(() -> joystick.stopRumble())
        );
    }

}
