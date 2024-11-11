package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Pigeon extends SubsystemBase {
    Pigeon2 pigeon;
    public Pigeon() {
        pigeon= new Pigeon2(0);
    }
    public double getYaw() {
        return pigeon.getYaw().getValue();
    }
    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }

    @Override
    public void periodic() {
        RobotState.getInstance().setHeading(getRotation2d());

    }
}
