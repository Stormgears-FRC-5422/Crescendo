package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SparkMax;
import frc.robot.RobotState;
import frc.utils.motorcontrol.LimitSwitch;
import frc.utils.vision.NoteVisualizer;

import static frc.robot.subsystems.Shooter.Direction.FORWARD;
import static frc.robot.subsystems.Shooter.Direction.REVERSE;


public class Climber extends SubsystemBase {
    public enum ClimberState {
        BAD,
        IDLE,
        CLIMBING,
        DESCENDING,
        HOMING,
        HANGING
    }

    private final CANSparkMax climberLeadMotor;
    private final CANSparkMax climberFollowerMotor;

    private SparkLimitSwitch climberForwardLimitSwitch;
    private SparkLimitSwitch climberZeroLimitSwitch;
    private final RobotState m_robotState;

    double m_climberMotorSpeed = 0;

    public Climber(){
        climberLeadMotor = new CANSparkMax(Constants.Climber.leaderID, CANSparkLowLevel.MotorType.kBrushless);
        climberFollowerMotor = new CANSparkMax(Constants.Climber.followerID, CANSparkLowLevel.MotorType.kBrushless);

        climberLeadMotor.setInverted(Constants.Climber.invertLeader);
        climberFollowerMotor.follow(climberLeadMotor, true);

        climberForwardLimitSwitch = climberLeadMotor.getForwardLimitSwitch(Type.kNormallyClosed);
        climberZeroLimitSwitch = climberLeadMotor.getReverseLimitSwitch(Type.kNormallyClosed);

        climberForwardLimitSwitch.enableLimitSwitch(true);
        climberZeroLimitSwitch.enableLimitSwitch(true);

        climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        m_robotState = RobotState.getInstance();
        setClimberState(ClimberState.IDLE);
    }

    @Override
    public void periodic() {
        climberLeadMotor.set(m_climberMotorSpeed);
    }

    public void setClimberState(Climber.ClimberState state) {
        m_robotState.setClimberState(state);

        switch (state) {
            case IDLE, BAD -> {
                m_climberMotorSpeed = 0;
            }
            case CLIMBING -> {
                m_climberMotorSpeed = Constants.Climber.climbSpeed;
            }
            case DESCENDING -> {
                m_climberMotorSpeed = -Constants.Climber.descendSpeed;
            }
            case HOMING -> {
                m_climberMotorSpeed = -Constants.Climber.homeSpeed;
            }
            case HANGING -> { // Separate from idle since we might need to do something else here like hold
                m_climberMotorSpeed = 0;
            }
            default -> System.out.println("invalid Climber state");
        }
    }

    public boolean isLockedIn(){
        return climberForwardLimitSwitch.isPressed();
    }

    public boolean isHome(){
        return climberZeroLimitSwitch.isPressed();
    }

    public void Stop(){
        setClimberState(ClimberState.IDLE);
    }

}
