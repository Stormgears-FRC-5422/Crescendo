package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;


public class Climber extends SubsystemBase {

    public enum ClimberState {
        BAD,     //There is an error
        IDLE_COAST,
        IDLE,    //When it is still and nothing is running(not necessarily at home position)
        CLIMBING, //When it is climbing onto the chain
        HOMING,  //When it is moving back to home
        HANGING, //It is on the chain
        HOME,     //It is at the home position
        MOVE_FORWARD, // moving to forward position
        MOVE_REVERSE  // moving to reverse position
    }

    public enum Direction {
        FORWARD,
        REVERSE
    }

    private final CANSparkMax climberLeadMotor;
    private final CANSparkMax climberFollowerMotor;

    private SparkLimitSwitch climberForwardLimitSwitch;
    private SparkLimitSwitch climberHomeLimitSwitch;
    private final RobotState robotState;
    private ClimberState myState;

    private boolean hasBeenHomed = false;
    private boolean hasSeenHome = false;
    private double homePosition;
    double m_climberMotorSpeed = 0;
    double m_targetPosition = 0;

    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private boolean doClimb;
    CANSparkMax.ControlType controlType;

    private double parkMin;
    private double parkMax;

    public Climber() {
        climberLeadMotor = new CANSparkMax(Constants.Climber.leaderID, CANSparkLowLevel.MotorType.kBrushless);
        climberLeadMotor.setInverted(Constants.Climber.invertLeader);
        climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        climberFollowerMotor = new CANSparkMax(Constants.Climber.followerID, CANSparkLowLevel.MotorType.kBrushless);
        climberFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        climberFollowerMotor.follow(climberLeadMotor, true);

        climberForwardLimitSwitch = climberLeadMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        climberHomeLimitSwitch = climberLeadMotor.getReverseLimitSwitch(Type.kNormallyClosed);

        climberForwardLimitSwitch.enableLimitSwitch(false);
        climberHomeLimitSwitch.enableLimitSwitch(false);

        parkMin = getPositionFromDegrees(Constants.Climber.parkedPositionDegrees
                                       - Constants.Climber.parkedToleranceDegrees);

        parkMax = getPositionFromDegrees(Constants.Climber.parkedPositionDegrees
                                       + Constants.Climber.parkedToleranceDegrees);

        setupPID();

        // Keep at the end of the constructor
        robotState = RobotState.getInstance();
        setClimberState(ClimberState.IDLE_COAST);

        climberLeadMotor.enableVoltageCompensation(12);
    }

    @Override
    public void periodic() {
        robotState.setClimberIsHome(isHome());

        if (!hasBeenHomed && isHome()) {
            hasSeenHome = true;
            homePosition = getPosition();
        }

        double pos = getPosition();
        robotState.setClimberIsParked(hasSeenHome &&
                                      pos > parkMin + homePosition &&
                                      pos < parkMax + homePosition );

        if (doClimb) {
//            m_pidController.setReference(getRPMFromPctOutput(m_climberMotorSpeed),
//                                         CANSparkMax.ControlType.kVelocity);
            double maxVoltage = 1.0;
            double theta = Math.toRadians(getDegreesFromPosition(m_encoder.getPosition()));
//            System.out.println("Theta is " + theta);
//            climberLeadMotor.set(maxVoltage * Math.abs(Math.cos(theta)));
            climberLeadMotor.set(1);
        } else {
            climberLeadMotor.set(m_climberMotorSpeed);
        }

    }

    private void setupPID() {
        m_pidController = climberLeadMotor.getPIDController();

        // Encoder object created to display position values
        m_encoder = climberLeadMotor.getEncoder();
//        climberFollowerMotor.burnFlash();
        // PID coefficients
        //kP = 0.05;
        kP = 0;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.00018;
//        kFF = Constants.Climber.climbSpeed * Constants.SparkMax.FreeSpeedRPM;

        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = Constants.SparkMax.FreeSpeedRPM;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void setClimberState(Climber.ClimberState state) {
        myState = state;
        robotState.setClimberState(state);

        // Almost all states need this
        climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        climberFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        switch (state) {
            case IDLE_COAST, BAD -> {
                m_climberMotorSpeed = 0;
                enableForwardSoftLimit(false);
                enableReverseSoftLimit(false);
                climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
                climberFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
                doClimb = false;
            }
            case IDLE -> {
                m_climberMotorSpeed = 0;
//                enableForwardSoftLimit(false);
//                enableReverseSoftLimit(false);
                doClimb = false;
            }
            case CLIMBING -> {
                setForwardSoftLimit((float) getPositionFromDegrees(Constants.Climber.climbStopInDegree));
                enableForwardSoftLimit(true);
//                m_climberMotorSpeed = Constants.Climber.climbSpeed;
                doClimb = true;
                climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                climberFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

            }
            case HOMING -> {
                enableForwardSoftLimit(false);
                enableReverseSoftLimit(false);
                m_climberMotorSpeed = -Constants.Climber.homeSpeed;
                doClimb = false;
                climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                climberFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
            }
            case HANGING -> { // Separate from idle since we might need to do something else here like hold
                m_climberMotorSpeed = 0;
                doClimb = false;
            }
            case HOME -> {//when it is actually home and done moving
                hasSeenHome = true;
                homePosition = 0;
                hasBeenHomed = true;
                m_encoder.setPosition(0);

                m_climberMotorSpeed = 0;
                doClimb = false;
            }
            case MOVE_FORWARD -> {    // climber is moving to amp shoot position
                climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                climberFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                m_climberMotorSpeed = Constants.Climber.fwdPosSpeed;
                setForwardSoftLimit((float) m_targetPosition);
                enableForwardSoftLimit(true);
                doClimb = false;
            }
            case MOVE_REVERSE -> {    // climber is moving to amp shoot position
                climberLeadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                climberFollowerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
                m_climberMotorSpeed = -1 * Constants.Climber.revPosSpeed;
                setReverseSoftLimit((float) m_targetPosition);
                enableReverseSoftLimit(true);
                doClimb = false;
            }
            default -> System.out.println("invalid Climber state");
        }
    }

    public void setTargetAngle(double angle) {
        m_targetPosition = getPositionFromDegrees(angle);
        System.out.println("Setting target angle to " + angle + ", position = " + m_targetPosition);
    }

    private double getRPMFromPctOutput(double percent) {
        return percent * Constants.SparkMax.FreeSpeedRPM;
    }

    public double getPositionFromDegrees(double degrees) {
        return (Constants.Climber.gearRatio / 360.0 ) *
               (degrees +
                Constants.Climber.offsetDegrees +
                Constants.Climber.overshootDegrees);
    }

    public double getDegreesFromPosition(double position) {
        double tmpDegrees = position / (Constants.Climber.gearRatio / 360.0 );
        return tmpDegrees - Constants.Climber.offsetDegrees - Constants.Climber.overshootDegrees;
    }

    public boolean isLockedIn() {
        return climberForwardLimitSwitch.isPressed();
    }

    public boolean isHome() {
        return climberHomeLimitSwitch.isPressed();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        setClimberState(ClimberState.IDLE);
    }

    public boolean isIdle() {
        return (robotState.getClimberState() == ClimberState.IDLE);
    }

    public boolean reachedTargetDegrees() {
        double error = m_targetPosition - m_encoder.getPosition();

//        System.out.println("error position: " + error);
//        System.out.println("error position: " + error);
//        System.out.println("ENCODER POSITION: " + m_encoder.getPosition());

        if (myState == ClimberState.MOVE_FORWARD){
            return error < 0;
        } else if (myState == ClimberState.MOVE_REVERSE) {
            return error > 0;
        } else {
            return false;
        }
    }

    public void setForwardSoftLimit(float limit) {
        climberLeadMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,limit);
    }
    public void setReverseSoftLimit(float limit) {
        climberLeadMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,limit);
    }

    public void enableForwardSoftLimit(boolean enable) {
        climberLeadMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, enable);
    }
    public void enableReverseSoftLimit(boolean enable) {
        climberLeadMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, enable);
    }
}
