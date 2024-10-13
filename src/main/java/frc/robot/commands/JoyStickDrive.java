package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.ButtonBoard;
import frc.robot.RobotState;
import frc.robot.ShuffleboardConstants;
import frc.robot.joysticks.CrescendoJoystick;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.drive.DrivetrainBase.driveFlip;

public class JoyStickDrive extends StormCommand {
    private DrivetrainBase drivetrain;
    private final DoubleSupplier txSupplier;
    private final DoubleSupplier tySupplier;
    private final DoubleSupplier omegaSupplier;
    private final BooleanSupplier robotRelativeSupplier;
    private final DoubleSupplier turboSupplier;

    private RobotState m_state;
    private boolean m_finish = true;
    private boolean m_flipJoystick = false;
    private final SlewRateLimiter xScaleLimiter = new SlewRateLimiter(Constants.Drive.linearRateLimiter); //make it into a constant
    private final SlewRateLimiter yScaleLimiter = new SlewRateLimiter(Constants.Drive.linearRateLimiter);
    private final SlewRateLimiter omegaScaleLimiter = new SlewRateLimiter(Constants.Drive.turnRateLimiter);



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
            robotRelativeSupplier.getAsBoolean() ? "Robot Orientation" : "Field Orientation");
    }

    @Override
    public void initialize() {
        super.initialize();

        if (m_state.isAllianceMissing()) {
            this.log("Alliance is not set. Exiting command");
            m_finish = true;
        }

        m_flipJoystick = ButtonBoard.flipJoystickForRed && m_state.isAllianceRed();
        this.log("Joystick is " + (m_flipJoystick ? "" : "NOT") + " flipped for alliance");

        m_finish = false;
    }

    @Override
    public boolean isFinished() {
        return m_finish;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        // TODO - do we really want to check this every iteration?
        if (Constants.Toggles.outReach) {
            drivetrain.setDriveSpeedScale(Drive.outReachSpeedScale);
        } else {
            if (turboSupplier.getAsDouble() <= 0.2) {
                drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
            } else {
                drivetrain.setDriveSpeedScale(turboSupplier.getAsDouble());
            }
        }
        ChassisSpeeds speeds;
        boolean fieldRelative;
        if (Constants.Toggles.outReach) {
            fieldRelative = false;
        } else {
            fieldRelative = !robotRelativeSupplier.getAsBoolean();
        }

        double x = txSupplier.getAsDouble();
        double y = tySupplier.getAsDouble();
        double omega = omegaSupplier.getAsDouble();

        //System.out.println("X: " + x + " Y: " + y + "Omega: " + omega);

        // When on the red alliance, we want to have "forward" mean "move in the -X direction" and so on.
        // But only for field relative driving. Robot relative driving is always the same

        if (m_flipJoystick && fieldRelative && driveFlip) {
            // speeds = new ChassisSpeeds(-x, -y, omega);
            if (Constants.ButtonBoard.squarePath) {
                speeds = new ChassisSpeeds(xScaleLimiter.calculate(-x*Math.abs(-x)), yScaleLimiter.calculate(-y*Math.abs(-y)),
                    omegaScaleLimiter.calculate(omega*Math.abs(omega)));
            } else {
                speeds = new ChassisSpeeds(xScaleLimiter.calculate(-x), yScaleLimiter.calculate(-y),
                    omegaScaleLimiter.calculate(omega));

            }
        } else {
            //  speeds = new ChassisSpeeds(x, y, omega);
            if (Constants.ButtonBoard.squarePath) {
                speeds = new ChassisSpeeds(xScaleLimiter.calculate(x*Math.abs(x)), yScaleLimiter.calculate(y*Math.abs(y)),
                    omegaScaleLimiter.calculate(omega*Math.abs(omega)));

            } else {
                speeds = new ChassisSpeeds(xScaleLimiter.calculate(x), yScaleLimiter.calculate(y),
                    omegaScaleLimiter.calculate(omega));
            }
        }
        drivetrain.percentOutputDrive(speeds, fieldRelative);
    }
}
