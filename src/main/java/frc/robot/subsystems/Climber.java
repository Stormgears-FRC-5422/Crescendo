package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;


public class Climber extends SubsystemBase {
    public enum ClimberState {
        BAD,     //There is an error
        IDLE,    //When it is still and nothing is running(not necessarily at home position)
        CLIMBING, //When it is climbing onto the chain
        HOMING,  //When it is moving back to home
        HANGING, //It is on the chain
        HOME     //It is at the home position
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

//        climberForwardLimitSwitch.enableLimitSwitch(true);
//        climberZeroLimitSwitch.enableLimitSwitch(true);

        climberForwardLimitSwitch.enableLimitSwitch(false);
        climberZeroLimitSwitch.enableLimitSwitch(false);

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

            case HOMING -> {
                m_climberMotorSpeed = -Constants.Climber.homeSpeed;
            }
            case HANGING -> { // Separate from idle since we might need to do something else here like hold
                m_climberMotorSpeed = 0;
            }
            case HOME -> {//when it is actually home and done moving
                m_climberMotorSpeed = 0;
            }
            default -> System.out.println("invalid Climber state");
        }
    }

    public boolean isLockedIn(){
//        return climberForwardLimitSwitch.isPressed();
        return false;
    }

    public boolean isHome(){
        return climberZeroLimitSwitch.isPressed();
    }

    public void stop(){
        setClimberState(ClimberState.IDLE);
    }
    public boolean isIdle(){
        return(m_robotState.getClimberState()==ClimberState.IDLE);
    }

}
