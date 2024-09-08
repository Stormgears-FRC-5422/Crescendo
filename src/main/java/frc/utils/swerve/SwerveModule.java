package frc.utils.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Conversions;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.utils.Conversions.MPSToRPS;


public class SwerveModule extends SubsystemBase {
    private final int moduleNumber;
    private final double angleOffset;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    //    private MotorController angleMotor;
//    private MotorController driveMotor;
    private CANcoder angleEncoder;
    @AutoLogOutput(key = "SwerveModule/Module {moduleNumber}/target angle")
    private double targetAngle;
    private double targetVelocity;

    private ControlRequest rotationDemand;
    private ControlRequest driveDemand;

    private double rotationVelocity;
    private double driveVelocity;
    private double drivePosition;


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, CANcoder cancoder) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        angleEncoder = cancoder;

//        if (Constants.Drive.motorControllerType.equals("Talon")) {
//            angleMotor = new StormTalon(moduleConstants.angleMotorID);
//            driveMotor = new StormTalon(moduleConstants.driveMotorID);
//        } else {
//            angleMotor = new StormSpark(moduleConstants.angleMotorID, CANSparkLowLevel.MotorType.kBrushless, StormSpark.MotorKind.k550);
//            driveMotor = new StormSpark(moduleConstants.driveMotorID, CANSparkLowLevel.MotorType.kBrushless, StormSpark.MotorKind.k550);
//        }

        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        driveMotor = new TalonFX(moduleConstants.driveMotorID);

        angleMotor.getConfigurator().apply(SwerveConstants.AngleFXConfig());
        driveMotor.getConfigurator().apply(SwerveConstants.DriveFXConfig());

//        angleMotor.applyConfig();
//        driveMotor.applyConfig();

        angleMotor.setInverted(false);


        driveMotor.setPosition(0);
        angleMotor.setPosition(0);

//        magnetSensorConfigs.MagnetOffset = 0;
//        angleEncoder.getConfigurator().apply(magnetSensorConfigs);

//        resetToAbsolute();

    }


//    public void resetToAbsolute() {
////        double angle = placeInAppropriate0To360Scope(getCurrentDegrees(), getAbsolutePosition() - angleOffset);
////        double absPosition = Conversions.degreesToRotation(angle, Constants.Drive.angleGearRatio);
//        angleEncoder.getAbsolutePosition().waitForUpdate(1);
//        double absPosition = Conversions.degreesToRotation(getAbsolutePosition() - angleOffset, Constants.Drive.angleGearRatio);
//        Logger.recordOutput("reset abs " + moduleNumber, absPosition);
//        StatusCode statusCode = angleMotor.setPosition(absPosition);
//        Logger.recordOutput("Sts code" + moduleNumber, statusCode);
//        Logger.recordOutput("reset " + moduleNumber, getAbsolutePosition()-angleOffset);
////
//
////        angleMotor.setPosition(Conversions.degreesToRotation(getCurrentDegrees(), Constants.Drive.angleGearRatio));
//    }

    public void resetToAbsolute() {
//        double angle = placeInAppropriate0To360Scope(getCurrentDegrees(), getAbsolutePosition() - angleOffset);
//        double absPosition = Conversions.degreesToRotation(angle, Constants.Drive.angleGearRatio);
        angleEncoder.getAbsolutePosition().waitForUpdate(1);
        double absPosition = Conversions.degreesToRotation(getAbsolutePosition(), Constants.Drive.angleGearRatio);
        angleMotor.setPosition(absPosition);
//

//        angleMotor.setPosition(Conversions.degreesToRotation(getCurrentDegrees(), Constants.Drive.angleGearRatio));
    }

    @AutoLogOutput(key = "SwerveModule/Module {moduleNumber}/absolute angle")
    public double getAbsolutePosition() {
        Logger.recordOutput("abs" + moduleNumber, angleEncoder.getAbsolutePosition().getValue() * 360);
        return angleEncoder.getAbsolutePosition().getValue() * 360;
    }

    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    @AutoLogOutput(key = "SwerveModule/Module {moduleNumber}/angle")
    public double getCurrentDegrees() {
        angleMotor.getRotorPosition().refresh();
        angleMotor.getRotorVelocity().refresh();
        double rotorPosition = BaseStatusSignal.getLatencyCompensatedValue(angleMotor.getRotorPosition(), angleMotor.getRotorVelocity());
//        return normalizeAngle(Conversions.rotationsToDegrees(rotorPosition, Constants.Drive.angleGearRatio));
        return Conversions.rotationsToDegrees(rotorPosition, Constants.Drive.angleGearRatio);
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360; // Ensure the angle is within 0 to 360
        if (angle > 180) {
            angle -= 360; // Convert angles greater than 180 to negative equivalents
        } else if (angle < -180) {
            angle += 360; // Convert angles less than -180 to positive equivalents
        }
        return angle;
    }

    public void setVelocity(SwerveModuleState desiredState) {
        Logger.recordOutput("desiredAngle" + moduleNumber, desiredState.angle.getDegrees());
        double flip = setSteeringAngleOptimized(desiredState.angle) ? -1 : 1;
        Logger.recordOutput("flip" + moduleNumber, flip);
        flip = 1;
//        setSteeringAngleRaw(desiredState.angle.getDegrees());
        targetVelocity = desiredState.speedMetersPerSecond * flip;
        double rotorSpeed = MPSToRPS(
            targetVelocity,
            Constants.Drive.wheelDiameter * Math.PI,
            Constants.Drive.driveGearRatio);

        if (Math.abs(rotorSpeed) < 0.002) {
            driveDemand = new NeutralOut();
        } else {
            driveDemand = new VelocityVoltage(rotorSpeed);
        }
    }

    private boolean setSteeringAngleOptimized(Rotation2d steerAngle) {
        boolean flip = false;

        final double targetClamped = steerAngle.getDegrees();
        final double angleUnclamped = getCurrentDegrees();
        final Rotation2d angleClamped = Rotation2d.fromDegrees(angleUnclamped);
        final Rotation2d relativeAngle = Rotation2d.fromDegrees(targetClamped).rotateBy(Conversions.inverse(angleClamped));
        double relativeDegrees = Math.round(relativeAngle.getDegrees());
        Logger.recordOutput("relativeDegrees" + moduleNumber, relativeDegrees);
        if (relativeDegrees > 90.0) {
            relativeDegrees -= 180.0;
            flip = true;

        } else if (relativeDegrees < -90.0) {
            relativeDegrees += 180.0;
            flip = true;
        }
        setSteeringAngleRaw(angleUnclamped + relativeDegrees);
        targetAngle = angleUnclamped + relativeDegrees;
        return flip;
    }

//    private boolean flipHeading(Rotation2d prevToGoal) {
//        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
//    }


    private void setSteeringAngleRaw(double angleDegrees) {
//        System.out.println(angleDegrees);
        double rotorPosition = Conversions.degreesToRotation(angleDegrees, Constants.Drive.angleGearRatio);

//        System.out.println(rotorPosition);
//        PositionVoltage positionVoltage = new PositionVoltage(rotorPosition);
        Logger.recordOutput("rotor pos " + moduleNumber, rotorPosition);
        rotationDemand = new PositionDutyCycle(rotorPosition, 0.0, false, 0.0, 0, false, false, false);
//        rotationDemand = new VoltageOut(rotorPosition* kV);
//        rotationDemand = positionVoltage;
    }

    @AutoLogOutput(key = "SwerveModule/Module {moduleNumber}/velocity")
    private double getCurrentVelocity() {
        return Conversions.RPSToMPS(
            driveVelocity,
            Constants.Drive.wheelDiameter * Math.PI,
            Constants.Drive.driveGearRatio);
    }


    public void setDriveNeutralBrake(boolean wantBrake) {
        TalonFXConfiguration t = new TalonFXConfiguration();
        driveMotor.getConfigurator().refresh(t);
        t.MotorOutput.NeutralMode = wantBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(t);
        angleMotor.getConfigurator().refresh(t);
        t.MotorOutput.NeutralMode = !wantBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        angleMotor.getConfigurator().apply(t);
    }


    public void outputTelemetry() {
        SmartDashboard.putNumber("Module" + moduleNumber + "/Azi Target", targetAngle);
        SmartDashboard.putNumber("Module" + moduleNumber + "/Azi Angle", getCurrentDegrees());
        SmartDashboard.putNumber("Module" + moduleNumber + "/Azi Error", getCurrentDegrees() - targetAngle);
        SmartDashboard.putNumber("Module" + moduleNumber + "/Wheel Velocity", Math.abs(getCurrentVelocity()));
        SmartDashboard.putNumber("Module" + moduleNumber + "/Wheel Target Velocity", Math.abs(targetVelocity));
        SmartDashboard.putNumber("Module" + moduleNumber + "/Drivetrain Position", Math.abs(drivePosition));
        SmartDashboard.putNumber("Module" + moduleNumber + "/Duty Cycle",
            driveMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Module" + moduleNumber + "/Azi Current",
            angleMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Module" + moduleNumber + "/Drivetrain Current",
            driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Module" + moduleNumber + "/Wheel Velocity Error",
            Math.abs(getCurrentVelocity()) - Math.abs(targetVelocity));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getCurrentVelocity(), Rotation2d.fromDegrees(getCurrentDegrees()));
    }

    @AutoLogOutput(key = "SwerveModule/Module {moduleNumber}/module angle")
    public double getModuleAngle() {
        return Rotation2d.fromDegrees(getCurrentDegrees()).getDegrees();
    }

    @Override
    public void periodic() {
        drivePosition = driveMotor.getRotorPosition().getValueAsDouble();
        rotationVelocity = angleMotor.getRotorVelocity().getValue();
        driveVelocity = driveMotor.getRotorVelocity().getValue();

//        Logger.recordOutput("Module" + moduleNumber + " rotor position", rotorPosition);

//        System.out.println(rotationDemand.getControlInfo());
        angleMotor.setControl(rotationDemand);
//        angleMotor.setPosition()
        driveMotor.setControl(driveDemand);
        rotationDemand = new NeutralOut();
        driveDemand = new NeutralOut();

    }

    public static class SwerveModuleConstants {
        public final int driveMotorID;
        public final int angleMotorID;
        public final double angleOffset;

        public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleOffset) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.angleOffset = angleOffset;
        }
    }

}
