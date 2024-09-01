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
        SWITCH_HOMING,  //When it is moving back to home
        CURRENT_HOMING,  //When it is moving back to home
        HANGING, //It is on the chain
        HOME,     //It is at the home position
        MOVE_FORWARD, // moving to forward position
        MOVE_REVERSE,  // moving to reverse position
        MOVE_PID_POSITION
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
    private double m_currentPosition;

    // STATE MACHINE
    private ClimberState myState;
    private boolean m_updateControllerState = true;
    private int m_currentLimit = 0;
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
    double m_motorSpeed = 0;
    double m_stallCurrent = 0;
    double m_targetPosition = 0;

    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    CANSparkMax.ControlType controlType;

    private final double armInitLowPosition;
    private final double armInitHighPosition;

    /**
     * constructs a new climber system & initializes motors
     *
     *
     */

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

        m_currentLimit = Constants.Climber.preHomeCurrentLimit;
        leadMotor.setSmartCurrentLimit(m_currentLimit);
        followerMotor.setSmartCurrentLimit(m_currentLimit);

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
    /**
     * This method is called periodically by the scheduler.
     */

    @Override
    public void periodic() {
        m_currentPosition = m_encoder.getPosition();
        boolean home = isHome();
        robotState.setClimberIsHome(home);

        // We want to indicate whether the arm is in the correct position before autonomous runs
        if (robotState.getPeriod() == RobotState.StatePeriod.DISABLED) {
            leadMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
            followerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
            if (!hasBeenHomed && home) {
                hasSeenHome = true;
                homePosition = m_currentPosition;
            }
            robotState.setClimberIsAtInit(hasSeenHome &&
                m_currentPosition >= homePosition + armInitLowPosition &&
                m_currentPosition <= homePosition + armInitHighPosition );
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

            leadMotor.setSmartCurrentLimit(m_currentLimit);
            followerMotor.setSmartCurrentLimit(m_currentLimit);

            m_updateControllerState = false;
        }

        // Control the motors here
        switch(myState) {
            case CLIMBING -> {
                // Just got for it
                leadMotor.set(m_motorSpeed);
            }
            case MOVE_PID_POSITION -> {
                double error = m_targetPosition - m_currentPosition;
                double arbitraryFF = getArbitraryFeedforward(error);
                System.out.println("error: " + getDegreesFromPosition(error) +
                                    ", angle: " + getDegreesFromPosition(m_currentPosition) +
                                    ", arbFF: " +  arbitraryFF);
                m_pidController.setReference(m_targetPosition, CANSparkMax.ControlType.kPosition, 0,
                    arbitraryFF, SparkPIDController.ArbFFUnits.kPercentOut);
            }
            default -> {
                leadMotor.set(m_motorSpeed);
            }
        }
    }

    /**
     * This method is called periodically by the scheduler.
     */

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

        // Natural current limits. Can be overridden below
        if(hasBeenHomed) {
            m_currentLimit = Constants.Climber.normalCurrentLimit;
        } else {
            m_currentLimit = Constants.Climber.preHomeCurrentLimit;
        }

        switch (state) {
            case IDLE_BRAKE -> {
                m_motorSpeed = 0;
                m_idleMode = CANSparkBase.IdleMode.kBrake;
            }
            case IDLE_COAST -> {
                m_motorSpeed = 0;
            }
            case CLIMBING -> {
                m_motorSpeed = getSpeedForDirection(Constants.Climber.climbSpeed, Direction.REVERSE);
                m_currentLimit = Constants.Climber.climbCurrentLimit;
                m_idleMode = CANSparkBase.IdleMode.kBrake;
                m_enableClimbLimitSwitch = true;
                m_targetPosition = getPositionFromDegrees(Constants.Climber.climbStopInDegrees);
                m_reverseSoftLimitPosition = m_targetPosition;
                m_enableReverseSoftLimit = true;
            }
            case HANGING -> { // Separate from idle since we might need to do something else here like hold
                m_motorSpeed = 0;
            }
            case SWITCH_HOMING -> {
                m_motorSpeed = getSpeedForDirection(Constants.Climber.homeSpeed, Direction.FORWARD);
                m_currentLimit = Constants.Climber.preHomeCurrentLimit;
                m_enableHomeLimitSwitch = true;
                m_idleMode = CANSparkBase.IdleMode.kBrake;
            }
            case CURRENT_HOMING -> {
                m_motorSpeed = getSpeedForDirection(Constants.Climber.homeSpeed, Direction.REVERSE);
                m_stallCurrent = Constants.Climber.stallCurrentLimit;
                m_idleMode = CANSparkBase.IdleMode.kBrake;
            }
            case HOME -> {//when it is actually home and done moving
                m_motorSpeed = 0;
                m_stallCurrent = 0;
                if (Constants.Climber.useCurrentLimitHomeStrategy) {
                    homePosition = getPositionFromDegrees(Constants.Climber.homeHardLimitDegrees);
                } else {
                    m_enableHomeLimitSwitch = true;
                    homePosition = getPositionFromDegrees(Constants.Climber.homeSwitchDegrees);
                }
                m_encoder.setPosition(homePosition);
                m_idleMode = CANSparkBase.IdleMode.kBrake;
                hasBeenHomed = true;
                hasSeenHome = true;
                robotState.setClimberHasBeenHomed(true);
            }
            case MOVE_FORWARD -> {    // climber is moving to amp shoot position
                m_currentLimit = Constants.Climber.normalCurrentLimit;

                m_motorSpeed = getSpeedForDirection(Constants.Climber.fwdPosSpeed, Direction.FORWARD);
                m_idleMode = CANSparkBase.IdleMode.kBrake;
                m_forwardSoftLimitPosition = m_targetPosition;
                m_enableForwardSoftLimit = true;
            }
            case MOVE_REVERSE -> {    // climber is moving to amp shoot position
                m_currentLimit = Constants.Climber.normalCurrentLimit;

                m_motorSpeed = getSpeedForDirection(Constants.Climber.revPosSpeed, Direction.REVERSE);
                m_idleMode = CANSparkBase.IdleMode.kBrake;
                m_reverseSoftLimitPosition = m_targetPosition;
                m_enableReverseSoftLimit = true;
            }
            case MOVE_PID_POSITION -> {
                m_currentLimit = Constants.Climber.normalCurrentLimit;

                m_idleMode = CANSparkBase.IdleMode.kBrake;
            }
            default -> System.out.println("invalid Climber state");
        }
    }

    private double getArbitraryFeedforward(double error) {
        double rampWidth = getPositionFromDegrees(Constants.Climber.pidTrapezoidRampDegrees);

        if (error > rampWidth) {
            return directionFactor * Constants.Climber.arbMaxFF;
        } else if (error < -rampWidth) {
            return -directionFactor * Constants.Climber.arbMaxFF;
        } else {
            return directionFactor * (error / rampWidth * (Constants.Climber.arbMaxFF - Constants.Climber.arbMinFF));
        }
    }

    private void setupPID() {
        m_pidController.setP(Constants.Climber.pidKp);
        m_pidController.setI(Constants.Climber.pidKi);
        m_pidController.setD(Constants.Climber.pidKd);
        m_pidController.setIZone(Constants.Climber.pidiZone);
        m_pidController.setFF(Constants.Climber.pidKf);
        m_pidController.setOutputRange(-Constants.Climber.pidMaxKout, Constants.Climber.pidMaxKout);
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

    public void setTargetDegrees(double degrees) {
        m_targetPosition = getPositionFromDegrees(degrees);
        System.out.println("Setting target to " + degrees + ", position = " + m_targetPosition);
        System.out.println("Current angle is " + getDegreesFromPosition(m_currentPosition) + ", position = " + m_currentPosition);
    }

    public boolean isLockedIn() {
        return climbLimitSwitch.isPressed();
    }

    public boolean isHome() {
        return homeLimitSwitch.isPressed();
    }

    public boolean hasStopped() {
        // weird tolerance in getVelocity - this appears to be due to long-term averaging within the sparkmax
        return Math.abs(m_encoder.getVelocity()) < 1 && leadMotor.get() == 0;
    }

    public double getPosition() {
        // Set in periodic
        return m_currentPosition;
    }

    public double getDegrees() {
        return getDegreesFromPosition(m_currentPosition);
    }

    public void stop() {
        setClimberState(ClimberState.IDLE_BRAKE);
    }

    public boolean isAtStallLimit() {
        return leadMotor.getOutputCurrent() > m_stallCurrent;
    }

    public boolean isIdle() {
        return (robotState.getClimberState() == ClimberState.IDLE_BRAKE);
    }

    public boolean reachedTargetDegrees() {
        double error = m_targetPosition - m_currentPosition;

        switch (myState) {
            case MOVE_PID_POSITION -> {
                return Math.abs(error) < getPositionFromDegrees(Constants.Climber.pidThresholdDegrees)
                       && Math.abs(m_encoder.getVelocity()) < 1;
            }
            case MOVE_FORWARD -> {
                return error < 0;
            }
            case MOVE_REVERSE -> {
                return error > 0;
            }
            default -> {
                return false;
            }
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
