package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.StateAlliance;
import frc.utils.lights.LEDLightStrip;
import frc.utils.lights.LightType;

import java.util.ArrayList;
import java.util.List;


public class StatusLights extends SubsystemBase {
    private static class Segment {
        private static int nextNumber = 0;
        final LightType lightType;
        final int numberOfLEDs;
        final int number;

        Segment(int numberOfLEDs, LightType lightType) {
            this.lightType = lightType;
            this.numberOfLEDs = numberOfLEDs;
            this.number = nextNumber++;
        }
    }

    private final RobotState m_robotState;
    private Shooter.ShooterState m_shooterState;
    private StateAlliance m_alliance;
    private int m_ledOffset;
    private int m_iteration;
    private Pose2d cameraTestPose = new Pose2d(15.0423, 5.467, Rotation2d.fromDegrees(0));

    public final Color8Bit RED_COLOR;
    public final Color8Bit GREEN_COLOR;
    public final Color8Bit BLUE_COLOR;
    public final Color8Bit ORANGE_COLOR;
    public final Color8Bit WHITE_COLOR;
    public final Color8Bit NO_COLOR = new Color8Bit(0,0,0);

    private Segment RING_TOP;
    private Segment RING_BOTTOM;
    private Segment RING_MIDDLE_TOP;
    private Segment RING_MIDDLE_BOTTOM;

    private LEDLightStrip m_ledLightStrip;
    boolean m_ledColorRequested;

    public StatusLights() {
        m_iteration = 0;
        m_robotState = RobotState.getInstance();

        m_shooterState = m_robotState.getShooterState();
        m_alliance = m_robotState.getAlliance();
        m_ledOffset = getCompassOffset();

        RED_COLOR = scaleColor(new Color8Bit(255, 0, 0), Constants.Lights.brightness);
        GREEN_COLOR = scaleColor(new Color8Bit(0, 255, 0), Constants.Lights.brightness);
        BLUE_COLOR = scaleColor(new Color8Bit(0, 0, 255), Constants.Lights.brightness);
        ORANGE_COLOR = scaleColor(new Color8Bit(255, 32, 0), Constants.Lights.brightness);
        WHITE_COLOR = scaleColor(new Color8Bit(84, 84, 84), Constants.Lights.brightness);

        initializeLights();
        setShooterLights();
        setCompass();
        System.out.println("Status Lights initializing ");
    }

    private static Color8Bit scaleColor(Color8Bit c, double s) {
        return new Color8Bit(MathUtil.clamp((int) (s * c.red), 0, 255),
                             MathUtil.clamp((int) (s * c.green), 0, 255),
                             MathUtil.clamp((int) (s * c.blue), 0, 255));
    }

    public void cameraAccuracy() {
        Transform2d poseError = new Transform2d(m_robotState.getVisionPose(), cameraTestPose);
//        System.out.println(poseError);
//        System.out.println("Translation: " + poseError);
//        System.out.println("Vision pose: " + RobotState.getInstance().getVisionPose());
        if (m_robotState.isVisionPoseValid()) {
            if (poseError.getX() < 1 && poseError.getY() < 1) {
                setRingColor(RING_TOP, GREEN_COLOR);
            } else {
                setRingColor(RING_TOP, RED_COLOR);
            }

            if (poseError.getRotation().getDegrees() < 10 && poseError.getRotation().getDegrees() > -10) {
                setRingColor(RING_MIDDLE_TOP, GREEN_COLOR);
            } else {
                setRingColor(RING_MIDDLE_TOP, RED_COLOR);
            }
        } else {
            setRingColor(RING_TOP, RED_COLOR);
            setRingColor(RING_MIDDLE_TOP, RED_COLOR);
        }
    }

    public void periodic() {

        cameraAccuracy();
        StateAlliance alliance = m_robotState.getAlliance();
        int offset = getCompassOffset();

        // Lights are expensive to check, and some updates can come too fast. Keep a counter to make updates less
        // frequent
        ++m_iteration;

        //System.out.println("alliance: " + alliance + ", m_alliance" + m_alliance);

        if (offset != m_ledOffset || alliance != m_alliance) {
            m_ledOffset = offset;
            m_alliance = alliance;
            setCompass();
        }

        if (m_shooterState != m_robotState.getShooterState()) {
            m_shooterState = m_robotState.getShooterState();
            setShooterLights();
        }

        if (m_ledColorRequested) {
            m_ledLightStrip.setLEDData();
        }
    }

    public void initializeLights() {
        System.out.println("init lights function");
        List<Segment> segments = new ArrayList<>();

        // These need to be added in the correct order. First string is closest to the roborio
        // Keep the assignment and segments.add together. This is necessary for the ring to have the right position
        RING_BOTTOM = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
        segments.add(RING_BOTTOM);

        RING_MIDDLE_BOTTOM = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
        segments.add(RING_MIDDLE_BOTTOM);

        RING_MIDDLE_TOP = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
        segments.add(RING_MIDDLE_TOP);

        RING_TOP = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
        segments.add(RING_TOP);

        m_ledLightStrip = new LEDLightStrip();
        for (Segment s : segments) {
            m_ledLightStrip.addSegment(s.numberOfLEDs, s.lightType);
        }
        m_ledLightStrip.setUp(Constants.Lights.port);
        m_ledColorRequested = true;
    }

    private void setShooterLights() {
        m_ledColorRequested = true;

        switch (m_shooterState) {
            case IDLE -> {
//                setRingColor(RING_TOP, WHITE_COLOR);
//                setRingColor(RING_MIDDLE_TOP, NO_COLOR);
                setRingColor(RING_MIDDLE_BOTTOM, WHITE_COLOR);
            }
            case STAGED_FOR_SHOOTING -> {
                setRingColor(RING_TOP, ORANGE_COLOR);
                setRingColor(RING_MIDDLE_TOP, ORANGE_COLOR);
                setRingColor(RING_MIDDLE_BOTTOM, ORANGE_COLOR);
            }
            case GROUND_PICKUP -> {
                setRingColor(RING_TOP, WHITE_COLOR);
                setAlternatingRingColor(RING_MIDDLE_TOP, WHITE_COLOR, ORANGE_COLOR);
                setRingColor(RING_MIDDLE_BOTTOM, ORANGE_COLOR);
            }
            case SOURCE_PICKUP_1, SOURCE_PICKUP_2 -> {
                setRingColor(RING_TOP, ORANGE_COLOR);
                setAlternatingRingColor(RING_MIDDLE_TOP, WHITE_COLOR, ORANGE_COLOR);
                setRingColor(RING_MIDDLE_BOTTOM, WHITE_COLOR);
            }
            case SPEAKER_SHOOTING -> {
                setRingColor(RING_TOP, GREEN_COLOR);
                setAlternatingRingColor(RING_MIDDLE_TOP, WHITE_COLOR, GREEN_COLOR);
                setRingColor(RING_MIDDLE_BOTTOM, WHITE_COLOR);
            }
            case AMP_SHOOTING -> {
                setRingColor(RING_TOP, WHITE_COLOR);
                setAlternatingRingColor(RING_MIDDLE_TOP, WHITE_COLOR, GREEN_COLOR);
                setRingColor(RING_MIDDLE_BOTTOM, GREEN_COLOR);
            }
            case DIAGNOSTIC -> {
                setAlternatingRingColor(RING_TOP, WHITE_COLOR, RED_COLOR);
                setAlternatingRingColor(RING_MIDDLE_TOP, RED_COLOR, BLUE_COLOR);
                setAlternatingRingColor(RING_MIDDLE_BOTTOM, BLUE_COLOR, WHITE_COLOR);
            }
            case OUTTAKE -> {
                setAlternatingRingColor(RING_TOP, ORANGE_COLOR, GREEN_COLOR);
                setAlternatingRingColor(RING_MIDDLE_TOP, GREEN_COLOR, ORANGE_COLOR);
                setAlternatingRingColor(RING_MIDDLE_BOTTOM, ORANGE_COLOR, GREEN_COLOR);
            }
            case BAD -> {
                setAlternatingRingColor(RING_TOP, WHITE_COLOR, ORANGE_COLOR);
                setAlternatingRingColor(RING_MIDDLE_TOP, ORANGE_COLOR, WHITE_COLOR);
                setAlternatingRingColor(RING_MIDDLE_BOTTOM, WHITE_COLOR, ORANGE_COLOR);
            }
            default -> {
                setAlternatingRingColor(RING_TOP, WHITE_COLOR, NO_COLOR);
                setAlternatingRingColor(RING_MIDDLE_TOP, NO_COLOR, WHITE_COLOR);
                setAlternatingRingColor(RING_MIDDLE_BOTTOM, WHITE_COLOR, NO_COLOR);
            }
        }
    }

    private void setAllianceLights() {
        m_ledColorRequested = true;
        switch (m_alliance) {
            case RED -> m_ledLightStrip.setLEDColor(RING_BOTTOM.number, RED_COLOR);
            case BLUE -> m_ledLightStrip.setLEDColor(RING_BOTTOM.number, BLUE_COLOR);
            default -> m_ledLightStrip.setAlternatingLEDColor(RING_BOTTOM.number, BLUE_COLOR, RED_COLOR);
        }
    }

    private void setRingColor(Segment s, Color8Bit c) {
        m_ledLightStrip.setLEDColor(s.number, c);
    }

    private void setAlternatingRingColor(Segment s, Color8Bit c1, Color8Bit c2) {
        m_ledLightStrip.setAlternatingLEDColor(s.number, c1, c2);
    }

    private int getCompassOffset() {
        // How far around the circle are we heading? Which led is that, offset from the front?
        // If the lights are wound clockwise or counterclockwise we need to change which way we go
        double degreesPerLight = 360.0 / Constants.Lights.oneRingLength;

        return (Constants.Lights.invertCompass ? -1 : 1) *
            (int) (Math.round(m_robotState.getHeading().getDegrees() / degreesPerLight));
    }

    private void setCompass() {
        // Start by setting the whole alliance ring
        setAllianceLights();

        // note that offsets might be positive or negative
        int ledNumber1 = (m_ledOffset + Constants.Lights.forwardLED + Constants.Lights.oneRingLength)
            % Constants.Lights.oneRingLength;
        int ledNumber2 = (m_ledOffset + Constants.Lights.backLEDOffset1 + Constants.Lights.forwardLED + Constants.Lights.oneRingLength)
            % Constants.Lights.oneRingLength;
        int ledNumber3 = (m_ledOffset + Constants.Lights.backLEDOffset2 + Constants.Lights.forwardLED + Constants.Lights.oneRingLength)
            % Constants.Lights.oneRingLength;

        Color8Bit pointerColor = switch (m_alliance) {
            case RED -> BLUE_COLOR;
            case BLUE -> RED_COLOR;
            default -> WHITE_COLOR;
        };

        m_ledLightStrip.setLEDColor(RING_BOTTOM.number, ledNumber1, pointerColor);
        m_ledLightStrip.setLEDColor(RING_BOTTOM.number, ledNumber2, pointerColor);
        m_ledLightStrip.setLEDColor(RING_BOTTOM.number, ledNumber3, pointerColor);
        m_ledColorRequested = true;
    }

}
