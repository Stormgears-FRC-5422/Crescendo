package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
import frc.robot.Constants.ButtonBoard;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class JoyStickDrive extends Command {
    private DrivetrainBase drivetrain;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier omegaSupplier;
    private final BooleanSupplier robotRelativeSupplier;
    private final BooleanSupplier turboSupplier;

    private RobotState m_state;
    private boolean m_finish = true;
    private boolean m_flipJoystick = false;

    public JoyStickDrive(DrivetrainBase drivetrain,
                         CrescendoJoystick joystick) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;

        m_state = RobotState.getInstance();

        txSupplier = joystick::getWpiX;
        tySupplier = joystick::getWpiY;
        omegaSupplier = joystick::getOmegaSpeed;
        robotRelativeSupplier = joystick::getRobotRelative;
        turboSupplier = joystick::getTurbo;

        ShuffleboardConstants.getInstance().drivetrainTab.add("Drive direction",
            robotRelativeSupplier.getAsBoolean() ? "Robot Orientation": "Field Orientation");
    }

    @Override
    public void initialize() {
        System.out.println("Starting Joystick Drive");

        if (m_state.isAllianceMissing()) {
            System.out.println("Alliance is not set. Exiting command");
            m_finish = true;
        }

        m_flipJoystick = ButtonBoard.flipJoystickForRed && m_state.isAllianceRed();
        System.out.println("Joystick is " + (m_flipJoystick ? "" : "NOT")+ " flipped for alliance");

        m_finish = false;
    }

    @Override
    public boolean isFinished() {
        return m_finish;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending Joystick Drive. Interrupted = " + interrupted);
    }

    @Override
    public void execute() {
        // TODO - do we really want to check this every iteration?
        if (!turboSupplier.getAsBoolean()) {
            drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
        } else {
            drivetrain.setDriveSpeedScale(Drive.driveSpeedScale);
        }

        ChassisSpeeds speeds;
        boolean fieldRelative = !robotRelativeSupplier.getAsBoolean();

        double x = txSupplier.getAsDouble();
        double y = tySupplier.getAsDouble();
        double omega = omegaSupplier.getAsDouble();

        //System.out.println("X: " + x + " Y: " + y + "Omega: " + omega);

        // When on the red alliance, we want to have "forward" mean "move in the -X direction" and so on.
        // But only for field relative driving. Robot relative driving is always the same
        if (m_flipJoystick && fieldRelative) {
            speeds = new ChassisSpeeds(-x, -y, omega);
        } else {
            speeds = new ChassisSpeeds(x, y, omega);
        }

        drivetrain.percentOutputDrive(speeds, fieldRelative);
    }
}
