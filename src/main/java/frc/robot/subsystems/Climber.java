package frc.robot.subsystems;

import com.revrobotics.*;
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
    private ClimberState m_myState;

    private boolean m_hasBeenHomed = false;
    double m_climberMotorSpeed = 0;

    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private double m_climbVelocity;
    private boolean m_usePid;

    public Climber() {
        climberLeadMotor = new CANSparkMax(Constants.Climber.leaderID, CANSparkLowLevel.MotorType.kBrushless);
        climberLeadMotor.setInverted(Constants.Climber.invertLeader);
        climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        climberFollowerMotor = new CANSparkMax(Constants.Climber.followerID, CANSparkLowLevel.MotorType.kBrushless);
        climberFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        climberFollowerMotor.follow(climberLeadMotor, true);

        climberForwardLimitSwitch = climberLeadMotor.getForwardLimitSwitch(Type.kNormallyClosed);
        climberZeroLimitSwitch = climberLeadMotor.getReverseLimitSwitch(Type.kNormallyClosed);

        climberForwardLimitSwitch.enableLimitSwitch(false);
        climberZeroLimitSwitch.enableLimitSwitch(false);

        setupPID();

        // Keep at the end of the constructor
        m_robotState = RobotState.getInstance();
        setClimberState(ClimberState.IDLE);
    }

    @Override
    public void periodic() {
        if (m_usePid) {
            m_pidController.setReference(m_climberMotorSpeed, CANSparkMax.ControlType.kVelocity);
        } else {
            climberLeadMotor.set(m_climberMotorSpeed);
        }
    }

    private void setupPID() {
        m_pidController = climberLeadMotor.getPIDController();

        // Encoder object created to display position values
        m_encoder = climberLeadMotor.getEncoder();

        // PID coefficients
        kP = 6e-5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        m_climbVelocity = 0.25;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }


    public void setClimberState(Climber.ClimberState state) {
        m_myState = state;
        m_robotState.setClimberState(state);

        switch (state) {
            case IDLE, BAD -> {
                m_climberMotorSpeed = 0;
                m_usePid = false;
            }
            case CLIMBING -> {
                m_climberMotorSpeed = Constants.Climber.climbSpeed;
                m_usePid = true;
            }
            case HOMING -> {
                m_climberMotorSpeed = -Constants.Climber.homeSpeed;
                m_usePid = false;
            }
            case HANGING -> { // Separate from idle since we might need to do something else here like hold
                m_climberMotorSpeed = 0;
                m_usePid = false;
            }
            case HOME -> {//when it is actually home and done moving
                m_hasBeenHomed = true;
                m_climberMotorSpeed = 0;
                m_usePid = false;
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
