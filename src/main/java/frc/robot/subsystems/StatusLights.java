package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    private Color8Bit[] compassRing;
    private Color8Bit[] visionXRing;
    private Color8Bit[] visionYRing;
    private Color8Bit[] visionRotationRing;

    private LEDLightStrip m_ledLightStrip;
    boolean m_ledColorRequested;

    private int ringLength;
    private int halfRingLength;

    public StatusLights() {
        m_iteration = 0;
        m_robotState = RobotState.getInstance();

        m_shooterState = m_robotState.getShooterState();
        m_alliance = m_robotState.getAlliance();

        RED_COLOR = scaleColor(new Color8Bit(255, 0, 0), Constants.Lights.brightness);
        GREEN_COLOR = scaleColor(new Color8Bit(0, 255, 0), Constants.Lights.brightness);
        BLUE_COLOR = scaleColor(new Color8Bit(0, 0, 255), Constants.Lights.brightness);
        ORANGE_COLOR = scaleColor(new Color8Bit(255, 32, 0), Constants.Lights.brightness);
        WHITE_COLOR = scaleColor(new Color8Bit(84, 84, 84), Constants.Lights.brightness);

        ringLength = Constants.Lights.oneRingLength;
        halfRingLength = Constants.Lights.oneRingLength / 2;

        compassRing = new Color8Bit[ringLength];
        visionXRing = new Color8Bit[ringLength];
        visionYRing = new Color8Bit[ringLength];
        visionRotationRing = new Color8Bit[ringLength];

        initializeLights();
        setShooterLights();
        makeCompassArray();
        System.out.println("Status Lights initializing ");
    }

    public void periodic() {
        // Lights may be expensive to check, and some updates can come too fast.
        // Keep a counter to make updates less frequent
        ++m_iteration;

        StateAlliance alliance = m_robotState.getAlliance();
        if (alliance != m_alliance) {
            m_alliance = alliance;
            makeCompassArray();
        }
        setSegmentFromColorArray(RING_BOTTOM, compassRing, m_robotState.getHeading());

        if (m_robotState.isUpperSensorTriggered()) {
            setRingColor(RING_MIDDLE_TOP, ORANGE_COLOR);
        } else {
            setRingColor(RING_MIDDLE_TOP, NO_COLOR);
        }

        // TODO - we need to decide when we want to display these.
        // assuming just during pre-game alignment for now
        if (m_robotState.getPeriod() == RobotState.StatePeriod.DISABLED) {
            cameraAccuracy();
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

    public void cameraAccuracy() {
        if (m_robotState.isVisionPoseValid()) {
            Transform2d poseError = new Transform2d(m_robotState.getVisionPose(), cameraTestPose);
    //        System.out.println(poseError);
    //        System.out.println("Translation: " + poseError);
    //        System.out.println("Vision pose: " + RobotState.getInstance().getVisionPose());
            getXYIndicatorRing(visionXRing, Math.abs(poseError.getX()));
            getXYIndicatorRing(visionYRing, Math.abs(poseError.getY()));
            getThetaIndicatorRing(visionRotationRing, Math.abs(poseError.getRotation().getDegrees()));

            setSegmentFromColorArray(RING_TOP, visionRotationRing, poseError.getRotation());
            setSegmentFromColorArray(RING_MIDDLE_TOP, visionXRing,
                new Rotation2d(poseError.getX() > 0 ? 1 : -1,0));
            setSegmentFromColorArray(RING_MIDDLE_BOTTOM, visionYRing,
                new Rotation2d(0, poseError.getY() > 0 ? 1 : -1));
        } else {
            setRingColor(RING_TOP, RED_COLOR);
            setRingColor(RING_MIDDLE_TOP, RED_COLOR);
            setRingColor(RING_MIDDLE_BOTTOM, RED_COLOR);
        }
    }

    private void getXYIndicatorRing(Color8Bit[] ring, double absError) {
        if (absError > 1.0) {
            for (int i=0; i < ringLength ; i++) {
                ring[i] = i < halfRingLength ? RED_COLOR : ORANGE_COLOR;
            }
        } else if (absError > 0.20) {
            for (int i=0; i < ringLength ; i++) {
                ring[i] = i < halfRingLength ? RED_COLOR : BLUE_COLOR;
            }
        } else if (absError > 0.02) {  // kinda cheesy, but OK.
            for (int i=0; i < ringLength ; i++) {
                ring[i] = i < absError / 0.02 ? RED_COLOR : WHITE_COLOR;
            }
        } else {
            for (int i=0; i < ringLength ; i++) {
                ring[i] = GREEN_COLOR;
            }
        }
    }

    private void getThetaIndicatorRing(Color8Bit[] ring, double absRotationError) {
        if (absRotationError > 45.0) {
            for (int i=0; i < ringLength ; i++) {
                ring[i] = i < halfRingLength ? RED_COLOR : BLUE_COLOR;
            }
        } else if (absRotationError > 10.0) {
            for (int i=0; i < ringLength ; i++) {
                ring[i] = i < halfRingLength ? RED_COLOR : ORANGE_COLOR;
            }
        } else if (absRotationError > 1.0) {  // kinda cheesy, but OK.
            for (int i=0; i < ringLength ; i++) {
                ring[i] = i < absRotationError / 1.0 ? RED_COLOR : WHITE_COLOR;
            }
        } else {
            for (int i=0; i < ringLength ; i++) {
                ring[i] = GREEN_COLOR;
            }
        }
    }

    private void setShooterLights() {
        m_ledColorRequested = true;

        switch (m_shooterState) {
            case IDLE -> {
                setRingColor(RING_TOP, WHITE_COLOR);
                //setRingColor(RING_MIDDLE_TOP, NO_COLOR);
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

    private static Color8Bit scaleColor(Color8Bit c, double s) {
        return new Color8Bit(MathUtil.clamp((int) (s * c.red), 0, 255),
            MathUtil.clamp((int) (s * c.green), 0, 255),
            MathUtil.clamp((int) (s * c.blue), 0, 255));
    }

    private void makeCompassArray() {
        for (int i = 0; i < Constants.Lights.oneRingLength; i++) {
            compassRing[i] = switch (m_alliance) {
                case RED -> RED_COLOR;
                case BLUE -> BLUE_COLOR;
                default -> (i % 2 == 0) ? BLUE_COLOR : RED_COLOR;
            };
        }

        Color8Bit pointerColor = switch (m_alliance) {
            case RED -> BLUE_COLOR;
            case BLUE -> RED_COLOR;
            default -> WHITE_COLOR;
        };

        compassRing[0] = pointerColor;
        compassRing[Constants.Lights.backLEDOffset1] = pointerColor;
        compassRing[Constants.Lights.backLEDOffset2] = pointerColor;
    }

    private void setRingColor(Segment s, Color8Bit c) {
        m_ledLightStrip.setLEDColor(s.number, c);
    }

    private void setAlternatingRingColor(Segment s, Color8Bit c1, Color8Bit c2) {
        m_ledLightStrip.setAlternatingLEDColor(s.number, c1, c2);
    }

    private void setSegmentFromColorArray(Segment s, Color8Bit[] colors, Rotation2d r) {
        int n = Constants.Lights.oneRingLength; // Total LEDs in one ring
        boolean invertRing = Constants.Lights.invertRingRotation; // True if the ring is inverted
        int angleShift = (int)Math.round(n * (r.getDegrees() / 360.0));

        // Pre-calculate shift based on inversion
        int shift = invertRing ? (n - angleShift) % n : angleShift % n;

        for (int i = 0; i < n; i++) {
            // Apply inversion to the index
            int indexAfterInversion = invertRing ? (n - 1 - i) : i;

            int newIndex = (indexAfterInversion + shift - Constants.Lights.forwardLEDIndex) % n;
            if (newIndex < 0) {
                newIndex += n; // Ensure newIndex is positive
            }
            m_ledLightStrip.setLEDColor(s.number, newIndex, colors[i]);
        }

        m_ledColorRequested = true;
    }

}
