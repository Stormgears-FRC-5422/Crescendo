package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;


public class Climber extends SubsystemBase {


    public enum ClimberState {
        IDLE_BRAKE,    //When it is still and nothing is running(not necessarily at home position)
        IDLE_COAST,
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

    // MOTORS
    private final CANSparkMax leadMotor;
    private final CANSparkMax followerMotor;
    private double directionFactor;

    private SparkLimitSwitch climbLimitSwitch;
    private SparkLimitSwitch homeLimitSwitch;
    private final RobotState robotState;


    // STATE MACHINE
    private ClimberState myState;
    private boolean m_updateControllerState = true;
    private boolean m_enableForwardSoftLimit;
    private boolean m_enableReverseSoftLimit;
    private double m_forwardSoftLimitPosition;
    private double m_reverseSoftLimitPosition;
    private boolean m_enableClimbLimitSwitch;
    private boolean m_enableHomeLimitSwitch;
    private CANSparkBase.IdleMode m_idleMode;
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

    private final double armInitLowPosition;
    private final double armInitHighPosition;

    public Climber() {
        leadMotor = new CANSparkMax(Constants.Climber.leaderID, CANSparkLowLevel.MotorType.kBrushless);
        leadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leadMotor.enableVoltageCompensation(12);
        leadMotor.setInverted(Constants.Climber.invertLeader);
        // FORWARD is defined by up-rotate-toward-front
        directionFactor = Constants.Climber.invertLeader ? -1.0 : 1.0;

        followerMotor = new CANSparkMax(Constants.Climber.followerID, CANSparkLowLevel.MotorType.kBrushless);
        followerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        followerMotor.follow(leadMotor, true);

        leadMotor.setSmartCurrentLimit(30);
        followerMotor.setSmartCurrentLimit(30);

        climbLimitSwitch = leadMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        homeLimitSwitch = leadMotor.getForwardLimitSwitch(Type.kNormallyOpen);

        double p1 = getPositionFromDegrees(Constants.Climber.initDegrees
                                         - Constants.Climber.initToleranceDegrees);
        double p2 = getPositionFromDegrees(Constants.Climber.initDegrees
                                         + Constants.Climber.initToleranceDegrees);

        armInitLowPosition = Math.min(p1, p2);
        armInitHighPosition = Math.max(p1, p2);

        m_pidController = leadMotor.getPIDController();
        m_encoder = leadMotor.getEncoder();
        setupPID();

        // Keep at the end of the constructor
        robotState = RobotState.getInstance();
        setClimberState(ClimberState.IDLE_COAST);
    }

    @Override
    public void periodic() {
        // if we were told to stop, stop first
        if (m_climberMotorSpeed == 0) {
            leadMotor.set(0);
        }

        boolean home = isHome();
        robotState.setClimberIsHome(home);

        // We want to indicate whether the arm is in the correct position before autonomous runs
        if (robotState.getPeriod() == RobotState.StatePeriod.DISABLED) {
            double position = getPosition();
            if (!hasBeenHomed && home) {
                hasSeenHome = true;
                homePosition = position;
            }

            robotState.setClimberIsAtInit(hasSeenHome &&
                position >= homePosition + armInitLowPosition &&
                position <= homePosition + armInitHighPosition );
        }

        // Don't need to reset these on every iteration
        if (m_updateControllerState) {
            setForwardSoftLimit((float)m_forwardSoftLimitPosition);
            setReverseSoftLimit((float)m_reverseSoftLimitPosition);
            enableForwardSoftLimit(m_enableForwardSoftLimit);
            enableReverseSoftLimit(m_enableReverseSoftLimit);
            leadMotor.setIdleMode(m_idleMode);
            followerMotor.setIdleMode(m_idleMode);
            climbLimitSwitch.enableLimitSwitch(m_enableClimbLimitSwitch);
            homeLimitSwitch.enableLimitSwitch(m_enableHomeLimitSwitch);

            m_updateControllerState = false;
        }

        if (doClimb) {
//            m_pidController.setReference(getRPMFromPctOutput(m_climberMotorSpeed),
//                                         CANSparkMax.ControlType.kVelocity);
            double maxVoltage = 1.0;
            double theta = Math.toRadians(getDegreesFromPosition(m_encoder.getPosition()));
//            System.out.println("Theta is " + theta);
//            climberLeadMotor.set(maxVoltage * Math.abs(Math.cos(theta)));
            leadMotor.set(-0.75);
        } else {
            leadMotor.set(m_climberMotorSpeed);
        }
    }

    public void setClimberState(Climber.ClimberState state) {
        myState = state;
        m_updateControllerState = true;
        robotState.setClimberState(state);

        // Almost all states need these values. Overwrite below as needed
        m_idleMode = CANSparkBase.IdleMode.kCoast;
        m_enableForwardSoftLimit = false;
        m_enableReverseSoftLimit = false;
        m_enableClimbLimitSwitch = false;
        m_enableHomeLimitSwitch = false;

        doClimb = false;

        switch (state) {
            case IDLE_BRAKE -> {
                m_climberMotorSpeed = 0;
                m_idleMode = CANSparkBase.IdleMode.kBrake;
            }
            case IDLE_COAST -> {
                m_climberMotorSpeed = 0;
            }
            case CLIMBING -> {
                m_climberMotorSpeed = getSpeedForDirection(Constants.Climber.climbSpeed, Direction.REVERSE);
                m_idleMode = CANSparkBase.IdleMode.kBrake;
                m_enableClimbLimitSwitch = true;
                m_targetPosition = getPositionFromDegrees(Constants.Climber.climbStopInDegrees);
                m_reverseSoftLimitPosition = m_targetPosition;
                m_enableReverseSoftLimit = true;
                doClimb = true;
            }
            case HANGING -> { // Separate from idle since we might need to do something else here like hold
                m_climberMotorSpeed = 0;
            }
            case HOMING -> {
                m_climberMotorSpeed = getSpeedForDirection(Constants.Climber.homeSpeed, Direction.FORWARD);
                m_enableHomeLimitSwitch = true;

                m_idleMode = CANSparkBase.IdleMode.kBrake;
            }
            case HOME -> {//when it is actually home and done moving
                m_climberMotorSpeed = 0;
                m_enableHomeLimitSwitch = true;
                homePosition = getPositionFromDegrees(Constants.Climber.homeDegrees);
                m_encoder.setPosition(homePosition);
                m_idleMode = CANSparkBase.IdleMode.kBrake;
                hasBeenHomed = true;
                hasSeenHome = true;
            }
            case MOVE_FORWARD -> {    // climber is moving to amp shoot position
                m_climberMotorSpeed = getSpeedForDirection(Constants.Climber.fwdPosSpeed, Direction.FORWARD);
                m_idleMode = CANSparkBase.IdleMode.kBrake;
                m_forwardSoftLimitPosition = m_targetPosition;
                m_enableForwardSoftLimit = true;
            }
            case MOVE_REVERSE -> {    // climber is moving to amp shoot position
                m_climberMotorSpeed = getSpeedForDirection(Constants.Climber.revPosSpeed, Direction.REVERSE);
                m_idleMode = CANSparkBase.IdleMode.kBrake;
                m_reverseSoftLimitPosition = m_targetPosition;
                m_enableReverseSoftLimit = true;
            }
            default -> System.out.println("invalid Climber state");
        }
    }

    private double getSpeedForDirection(double speed, Direction direction) {
        return direction == Direction.FORWARD ? directionFactor * speed : -directionFactor * speed;
    }

    public double getPositionFromDegrees(double degrees) {
        return directionFactor * degrees * (Constants.Climber.gearRatio / 360.0 );
    }

    public double getDegreesFromPosition(double position) {
        return directionFactor * position / (Constants.Climber.gearRatio / 360.0 );
    }

    private void setupPID() {
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = Constants.SparkMax.FreeSpeedRPM;

        kP = 0;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 1.0 / maxRPM;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void setTargetAngle(double angle) {
        m_targetPosition = getPositionFromDegrees(angle);
        System.out.println("Setting target angle to " + angle + ", position = " + m_targetPosition);
    }

    private double getRPMFromPctOutput(double percent) {
        return percent * Constants.SparkMax.FreeSpeedRPM;
    }

    public boolean isLockedIn() {
        return climbLimitSwitch.isPressed();
    }

    public boolean isHome() {
        return homeLimitSwitch.isPressed();
    }

    public boolean hasStopped() {
//        System.out.println("vel: " + m_encoder.getVelocity() + " get: " + climberLeadMotor.get());
        // weird tolerance in getVelocity
        return Math.abs(m_encoder.getVelocity()) < 1 && leadMotor.get() == 0;
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        setClimberState(ClimberState.IDLE_BRAKE);
    }

    public boolean isIdle() {
        return (robotState.getClimberState() == ClimberState.IDLE_BRAKE);
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
        leadMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward,limit);
    }
    public void setReverseSoftLimit(float limit) {
        leadMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,limit);
    }

    public void enableForwardSoftLimit(boolean enable) {
        leadMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, enable);
    }
    public void enableReverseSoftLimit(boolean enable) {
        leadMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, enable);
    }
}
