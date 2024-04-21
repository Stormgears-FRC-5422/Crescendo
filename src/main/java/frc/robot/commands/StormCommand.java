package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class StormCommand extends Command {
    protected String m_name = "StormCommand";

    public StormCommand() {
        m_name = getClass().getName();
        m_name = m_name.substring(m_name.lastIndexOf('.') + 1);
        this.log("created");
    }

    public void log(String message) {
        System.out.println("Command " + m_name + ": " + message);
    }

    @Override
    public void initialize()  {
        this.log("initialized");
    }

    @Override
    public void end(boolean interrupted) {
        this.log("end: interrupted = " + interrupted);
    }

}
