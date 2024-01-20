package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.Map;
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
                         DoubleSupplier txSupplier, DoubleSupplier tySupplier, DoubleSupplier omegaSupplier,
                         BooleanSupplier robotRelativeSupplier, BooleanSupplier turboSupplier) {
        this.drivetrain = drivetrain;
        this.txSupplier = txSupplier;
        this.tySupplier = tySupplier;
        this.omegaSupplier = omegaSupplier;
        this.robotRelativeSupplier = robotRelativeSupplier;
        this.turboSupplier = turboSupplier;

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
                                                        0);
//        omegaSupplier.getAsDouble()
        drivetrain.drive(chassisSpeeds, robotRelativeSupplier.getAsBoolean());
    }
}
