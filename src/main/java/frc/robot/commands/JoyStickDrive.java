package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
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

    public JoyStickDrive(DrivetrainBase drivetrain,
                         CrescendoJoystick joystick) {
        this.drivetrain = drivetrain;
        txSupplier = joystick::getWpiX;
        this.tySupplier = joystick::getWpiY;
        this.omegaSupplier = joystick::getOmegaSpeed;
        this.robotRelativeSupplier = joystick::getRobotRelative;
        this.turboSupplier = joystick::getTurbo;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!turboSupplier.getAsBoolean()) {
            drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
        } else {
            drivetrain.setDriveSpeedScale(Drive.driveSpeedScale);
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(txSupplier.getAsDouble(),
                                                        tySupplier.getAsDouble(),
                                                        omegaSupplier.getAsDouble());

        drivetrain.drive(chassisSpeeds, robotRelativeSupplier.getAsBoolean());
    }
}
