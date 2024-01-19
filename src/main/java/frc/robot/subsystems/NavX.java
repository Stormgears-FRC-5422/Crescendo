package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainBase.*;

import static frc.robot.Constants.navXConnection;

public class NavX extends SubsystemBase {

    private final AHRS m_gyro;

    public Rotation2d getAbsoluteRotation;

    public NavX() {
        switch (navXConnection) {
            case "SPI" -> m_gyro = new AHRS(SPI.Port.kMXP);
            case "USB" -> m_gyro = new AHRS(SerialPort.Port.kUSB);
            default -> {
                m_gyro = new AHRS(SPI.Port.kMXP);
                System.out.println("NO NavX Connection Given. Default NavX connection used: SPI");
            }
        }
        getAbsoluteRotation = Rotation2d.fromDegrees(MathUtil.inputModulus(getYaw(), 180, -180));
        resetHeading();
    }

    public double getYaw() {
        return m_gyro.getYaw();
    }

    public void resetHeading() {
        m_gyro.reset();
    }


    @Override
    public void periodic() {
        RobotState.getInstance().setGyroData(getAbsoluteRotation);
    }

}
