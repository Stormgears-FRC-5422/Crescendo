package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.drive.DrivetrainBase.*;

import static frc.robot.Constants.navXConnection;

public class NavX extends SubsystemBase {

    private final AHRS m_gyro;
    private double offset = 0.0;

    public NavX() {
        switch (navXConnection) {
            case "SPI":
                m_gyro = new AHRS(SPI.Port.kMXP);
                break;
            case "USB":
                m_gyro = new AHRS(SerialPort.Port.kUSB);
                break;
            default:
                m_gyro = new AHRS(SPI.Port.kMXP);
                System.out.println("NO NavX Connection Given. Default NavX connection used: SPI");
                break;
        }
    }


//        ShuffleboardTab tab = ShuffleboardConstants.getInstance().navXTab;
//        tab.addNumber("yaw", this::getYaw);
//        tab.addNumber("Absolute Yaw", () -> getAbsoluteRotation().getDegrees());
//        ShuffleboardConstants.getInstance().driverTab
//                .addDouble("Gyro", () -> getAbsoluteRotation().getDegrees())
////                .withWidget(BuiltInWidgets.kGyro)
//                .withPosition(3, 2).withSize(1, 1);
//    }



    public double getYaw() {
        return m_gyro.getYaw();
    }

    public double getPitch() {
        return m_gyro.getPitch();
    }

    public double getRoll() {
        return m_gyro.getRoll();
    }

    public boolean isMagnetometerCalibrated() {
        return m_gyro.isMagnetometerCalibrated();
    }

    public void zeroYaw() {
        m_gyro.zeroYaw();
        offset = 0.0;
    }



    /** get absolute rotation (-180, 180) inverted so counter-clockwise is positive with offset */
    public Rotation2d getAbsoluteRotation = Rotation2d.fromDegrees(MathUtil.inputModulus(getYaw(), 180, -180));




    @Override
    public void periodic() {
        DrivetrainBase.angle = getAbsoluteRotation;
    }
}
