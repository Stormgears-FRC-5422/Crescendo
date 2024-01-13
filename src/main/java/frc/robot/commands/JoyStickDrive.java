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

    private DrivetrainBase m_drivetrain;
    private final DoubleSupplier txSupplier, tySupplier, omegaSupplier;

    private final BooleanSupplier robotRelativeSupplier, turboSupplier;

    public JoyStickDrive(DrivetrainBase drivetrain,
                                     DoubleSupplier txSupplier, DoubleSupplier tySupplier, DoubleSupplier omegaSupplier,
                                     BooleanSupplier robotRelativeSupplier, BooleanSupplier turboSupplier) {
        m_drivetrain = drivetrain;
        this.txSupplier = txSupplier;
        this.tySupplier = tySupplier;
        this.omegaSupplier = omegaSupplier;
        this.robotRelativeSupplier = robotRelativeSupplier;
        this.turboSupplier = turboSupplier;


        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        if (!turboSupplier.getAsBoolean())
            m_drivetrain.setDriveSpeedScale(Drive.precisionSpeedScale);
        else
            m_drivetrain.setDriveSpeedScale(Drive.driveSpeedScale);

        m_drivetrain.drive(
                new ChassisSpeeds(txSupplier.getAsDouble(),tySupplier.getAsDouble(),omegaSupplier.getAsDouble()),
                robotRelativeSupplier.getAsBoolean()
        );
    }
}
