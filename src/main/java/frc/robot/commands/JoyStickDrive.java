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

    private boolean m_flipJoystick = false;

    public JoyStickDrive(DrivetrainBase drivetrain,
                         CrescendoJoystick joystick) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;

        txSupplier = joystick::getWpiX;
        tySupplier = joystick::getWpiY;
        omegaSupplier = joystick::getOmegaSpeed;
        robotRelativeSupplier = joystick::getRobotRelative;
        turboSupplier = joystick::getTurbo;

        ShuffleboardConstants.getInstance().drivetrainTab.add("Drive direction",
            robotRelativeSupplier.getAsBoolean()? "Field Orientation": "Robot Orientation");
    }

    @Override
    public void initialize() {
        System.out.println("Starting Joystick Drive");
        m_flipJoystick = ButtonBoard.flipJoystickForRed && !RobotState.getInstance().isAllianceBlue();
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

        // When on the red alliance, we want to have "forward" mean "move in the -X direction" and so on.
        if (!m_flipJoystick) {
            speeds = new ChassisSpeeds(txSupplier.getAsDouble(), tySupplier.getAsDouble(),
                omegaSupplier.getAsDouble());
        } else {
            speeds = new ChassisSpeeds(-txSupplier.getAsDouble(), -tySupplier.getAsDouble(),
                omegaSupplier.getAsDouble());
        }

        drivetrain.percentOutputDrive(speeds, fieldRelative);
    }
}
