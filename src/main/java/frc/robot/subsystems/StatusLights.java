package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CrescendoField;
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
    public final Color8Bit YELLOW_COLOR;
    public final Color8Bit WHITE_COLOR;
    public final Color8Bit NO_COLOR = new Color8Bit(0, 0, 0);

    private Segment RING_TOP;
    private Segment RING_MIDDLE_TOP;
    private Segment RING_MIDDLE;
    private Segment RING_MIDDLE_BOTTOM;
    private Segment RING_BOTTOM;
    private Segment LEFT_SIDE_MAIN;
    private Segment RIGHT_SIDE_MAIN;
    private Segment LEFT_SIDE_TOP;
    private Segment RIGHT_SIDE_TOP;


    private Color8Bit[] compassRing;
    private Color8Bit[] visionXRing;
    private Color8Bit[] visionYRing;
    private Color8Bit[] visionRotationRing;
    private Color8Bit[] leftOrientationRing;
    private Color8Bit[] rightOrientationRing;
    public Color8Bit sideColor;
    public Color8Bit topColor;
    private Color8Bit allianceColor;
    private Color8Bit otherAllianceColor;

    private LEDLightStrip m_ledLightStrip;
    boolean m_ledColorRequested;

    private int ringLength;
    private int halfRingLength;
    private int quarterRingLength;

    List<Pose2d> targetList;
    VisionSubsystem visionSubsystem;
    int count = 0;


    public StatusLights(VisionSubsystem visionSubsystem) {
        m_iteration = 0;
        m_robotState = RobotState.getInstance();
        m_alliance = m_robotState.getAlliance();
        m_shooterState = m_robotState.getShooterState();

        this.visionSubsystem = visionSubsystem;

        RED_COLOR = scaleColor(new Color8Bit(255, 0, 0), Constants.Lights.brightness);
        GREEN_COLOR = scaleColor(new Color8Bit(0, 255, 0), Constants.Lights.brightness);
        BLUE_COLOR = scaleColor(new Color8Bit(0, 0, 255), Constants.Lights.brightness);
        ORANGE_COLOR = scaleColor(new Color8Bit(255, 32, 0), Constants.Lights.brightness);
        YELLOW_COLOR = scaleColor(new Color8Bit(255, 255, 0), Constants.Lights.brightness);
        WHITE_COLOR = scaleColor(new Color8Bit(84, 84, 84), Constants.Lights.brightness);
        setAllianceColor();

        ringLength = Constants.Lights.sideMainLength;
        halfRingLength = Constants.Lights.sideMainLength / 2;
        quarterRingLength = Constants.Lights.sideMainLength / 4;

//        compassRing = new Color8Bit[ringLength];
//        visionXRing = new Color8Bit[ringLength];
//        visionYRing = new Color8Bit[ringLength];
//        visionRotationRing = new Color8Bit[ringLength];
        leftOrientationRing = new Color8Bit[Constants.Lights.sideMainLength];
        rightOrientationRing = new Color8Bit[Constants.Lights.sideMainLength];

        initializeLights();
//        setShooterLights();
//        makeCompassArray();
        makeTargetList();
        System.out.println("Status Lights initializing ");
    }

    public void periodic() {
        // Lights may be expensive to check, and some updates can come too fast.
        // Keep a counter to make updates less frequent
        ++m_iteration;

        StateAlliance alliance = m_robotState.getAlliance();
        if (alliance != m_alliance) {
            m_alliance = alliance;
            setAllianceColor();
            //makeCompassArray();
            makeTargetList();
        }

        if (m_robotState.getPeriod() == RobotState.StatePeriod.DISABLED) {
            if (Constants.Toggles.outReach) {
                topColor = BLUE_COLOR;
                sideColor = BLUE_COLOR;
            } else {
                // Set light patterns for pre-match configuration
//            if (m_robotState.isClimberAtInit()) {
//                topColor = GREEN_COLOR;
//            } else {
//                topColor = WHITE_COLOR;
//            }

                if (m_robotState.isVisionPoseValid()) {
                    topColor = YELLOW_COLOR;
                } else {
                    topColor = WHITE_COLOR;
                }
            }
            setRingColor(LEFT_SIDE_TOP, topColor);
            setRingColor(RIGHT_SIDE_TOP, topColor);
            setRingColor(LEFT_SIDE_MAIN, sideColor);
            setRingColor(RIGHT_SIDE_MAIN, sideColor);
            // Main lights handled here
            if (!Constants.Toggles.outReach) {
                setAlignmentLights();
            }
        } else {

            // TOP lights
            // TODO - I don't like that this needs to both use robot state and have its own copy of the vision subsystem.
            if (m_robotState.isVisionPoseValid()) {
                topColor = YELLOW_COLOR;
            } else {
                topColor = WHITE_COLOR;
            }

            // Main lights
            if (m_robotState.isUpperSensorTriggered()) {
                sideColor = GREEN_COLOR;
            } else if (m_robotState.getIsNoteDetected()) {
                sideColor = ORANGE_COLOR;
            } else {
                sideColor = allianceColor;
            }


            if (count == 50) {
                count = 0;
            } else if (count < 25) {
                if (Constants.Toggles.outReach) {
                    setRingColor(LEFT_SIDE_TOP, sideColor);
                    setRingColor(RIGHT_SIDE_TOP, sideColor);
                    setRingColor(LEFT_SIDE_MAIN, sideColor);
                    setRingColor(RIGHT_SIDE_MAIN, sideColor);
                } else {
                    setRingColor(LEFT_SIDE_TOP, topColor);
                    setRingColor(RIGHT_SIDE_TOP, topColor);
                    setRingColor(LEFT_SIDE_MAIN, sideColor);
                    setRingColor(RIGHT_SIDE_MAIN, sideColor);
                }
            } else {
                setRingColor(LEFT_SIDE_TOP, NO_COLOR);
                setRingColor(RIGHT_SIDE_TOP, NO_COLOR);
                setRingColor(LEFT_SIDE_MAIN, NO_COLOR);
                setRingColor(RIGHT_SIDE_MAIN, NO_COLOR);
            }
            count++;

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

        LEFT_SIDE_TOP = new Segment(Constants.Lights.sideTopLength, LightType.getType(Constants.Lights.ringLEDType));
        segments.add(LEFT_SIDE_TOP);

        LEFT_SIDE_MAIN = new Segment(Constants.Lights.sideMainLength, LightType.getType(Constants.Lights.ringLEDType));
        segments.add(LEFT_SIDE_MAIN);

        RIGHT_SIDE_TOP = new Segment(Constants.Lights.sideTopLength, LightType.getType(Constants.Lights.ringLEDType));
        segments.add(RIGHT_SIDE_TOP);

        RIGHT_SIDE_MAIN = new Segment(Constants.Lights.sideMainLength, LightType.getType(Constants.Lights.ringLEDType));
        segments.add(RIGHT_SIDE_MAIN);

//        RING_TOP = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
//        segments.add(RING_TOP);
//
//        RING_MIDDLE_TOP = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
//        segments.add(RING_MIDDLE_TOP);
//
//        RING_MIDDLE = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
//        segments.add(RING_MIDDLE);
//
//        RING_MIDDLE_BOTTOM = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
//        segments.add(RING_MIDDLE_BOTTOM);
//
//
//        RING_BOTTOM = new Segment(Constants.Lights.oneRingLength, LightType.getType(Constants.Lights.ringLEDType));
//        segments.add(RING_BOTTOM);

        m_ledLightStrip = new LEDLightStrip();
        for (Segment s : segments) {
            m_ledLightStrip.addSegment(s.numberOfLEDs, s.lightType);
        }
        m_ledLightStrip.setUp(Constants.Lights.port);
        m_ledColorRequested = true;
    }

    public void makeTargetList() {
        targetList = new ArrayList<Pose2d>();

        targetList.add(CrescendoField.remapPose(new Pose2d(1.356, 5.553, new Rotation2d(1, 0)), m_alliance)); //middle
        targetList.add(CrescendoField.remapPose(new Pose2d(0.77, 4.393, new Rotation2d(0.5994, -0.8220)), m_alliance)); //speaker
        targetList.add(CrescendoField.remapPose(new Pose2d(0.768, 6.711, new Rotation2d(0.5694, 0.8220)), m_alliance)); // amp
    }

    public void setAlignmentLights() {
        Pose2d current = m_robotState.getVisionPose();
//        System.out.print("vision pose " + current);
        if (current == null || current.getX() == 0) {
            setAlternatingRingColor(RIGHT_SIDE_MAIN, GREEN_COLOR, WHITE_COLOR);
            setAlternatingRingColor(LEFT_SIDE_MAIN, GREEN_COLOR, WHITE_COLOR);
            return;
        }

        Pose2d target = current.nearest(targetList);
        Transform2d poseError = new Transform2d(current, target);
        //        System.out.println(poseError);
        //        System.out.println("Translation: " + poseError);
        //        System.out.println("Vision pose: " + RobotState.getInstance().getVisionPose());
        setLeftRightRings(leftOrientationRing, rightOrientationRing, poseError.getY());

        // We want to move up and down, but not wrap around. So we need to max the rotation and shift total at
        // just shy of 180 degrees - the top and bottom lights.
        // Move up and down based on X distance
        double xOffset = getXOffset(poseError.getX(), halfRingLength);
        double rotationOffset = getRotationOffset(poseError.getRotation(), halfRingLength);

        // The center light is already set based on the y alignment
        if (xOffset != 0 || rotationOffset != 0) {
            leftOrientationRing[halfRingLength] = allianceColor;
            rightOrientationRing[halfRingLength] = allianceColor;
        }

        // convert the offset to an angle
        double leftOffset = MathUtil.clamp(xOffset - rotationOffset, 0, ringLength);
        double rightOffset = MathUtil.clamp(xOffset + rotationOffset, 0, ringLength);
        ;

        // The function here pretends that the light string is a ring. So we need to convert the
        // offset to radians using this weird scale. Note that the end of the string isn't quite at 180
        double toRadians = Math.PI / (halfRingLength + 1);

        setSegmentFromColorArray(LEFT_SIDE_MAIN, leftOrientationRing, new Rotation2d(leftOffset * toRadians));
        setSegmentFromColorArray(RIGHT_SIDE_MAIN, rightOrientationRing, new Rotation2d(rightOffset * toRadians));
    }

    private double getRotationOffset(Rotation2d error, int maxOffset) {
        double absErrorDeg = Math.abs(error.getDegrees());
        boolean rotateClockwise = error.getDegrees() > 0;
        double offset;

        if (absErrorDeg > 30) {
            offset = maxOffset;
        } else if (absErrorDeg > 10) {
            offset = maxOffset / 2.0 + (absErrorDeg - 10) / 6.0;
        } else if (absErrorDeg > 2) {
            offset = absErrorDeg / 5.0;
        } else {
            offset = 0;
        }

        return rotateClockwise ? offset : -offset;
    }

    private double getXOffset(double error, int maxOffset) {
        double absErrorCm = Math.abs(100 * error);
        boolean moveForward = error > 0;
        double offset;

        if (absErrorCm > 30) {
            offset = maxOffset;
        } else if (absErrorCm > 10) {
            offset = maxOffset / 2.0 + (absErrorCm - 10) / 6.0;
        } else if (absErrorCm > 2) {
            offset = absErrorCm / 5.0;
        } else {
            offset = 0;
        }

        return moveForward ? offset : -offset;
    }

    private void setLeftRightRings(Color8Bit[] leftRing, Color8Bit[] rightRing, double error) {
        double absError = Math.abs(error);
        boolean moveRight = error > 0; // I think?
        int i;

        Color8Bit leftColor = moveRight ? allianceColor : NO_COLOR;
        Color8Bit rightColor = moveRight ? NO_COLOR : allianceColor;

        // Start out by clearing the entire segment
        for (i = 0; i < ringLength; i++) {
            leftRing[i] = NO_COLOR;
            rightRing[i] = NO_COLOR;
        }

        // Make an "arrow", with the bigger range of lights on the error side, and the central dot pointing to the other
        if (absError > 1.0) {
            for (i = 0; i < ringLength; i++) {
                leftRing[i] = leftColor;
            }
            leftRing[halfRingLength] = rightColor;
            rightRing[halfRingLength] = leftColor;
        } else if (absError > 0.20) {
            for (i = quarterRingLength; i < halfRingLength + quarterRingLength; i++) {
                leftRing[i] = leftColor;
                rightRing[i] = rightColor;
            }
            leftRing[halfRingLength] = rightColor;
            rightRing[halfRingLength] = leftColor;
        } else if (absError > 0.02) {  // kinda cheesy, but OK.
            leftRing[halfRingLength - 1] = leftColor;
            leftRing[halfRingLength] = rightColor;
            leftRing[halfRingLength + 1] = leftColor;
            rightRing[halfRingLength] = leftColor;
        } else {
            leftRing[halfRingLength] = GREEN_COLOR;
            rightRing[halfRingLength] = GREEN_COLOR;
        }
    }

    private void getXYIndicatorRing(Color8Bit[] ring, double absError) {
        if (absError > 1.0) {
            for (int i = 0; i < ringLength; i++) {
                ring[i] = i < halfRingLength ? RED_COLOR : ORANGE_COLOR;
            }
        } else if (absError > 0.20) {
            for (int i = 0; i < ringLength; i++) {
                ring[i] = i < halfRingLength ? RED_COLOR : BLUE_COLOR;
            }
        } else if (absError > 0.02) {  // kinda cheesy, but OK.
            for (int i = 0; i < ringLength; i++) {
                ring[i] = i < absError / 0.02 ? RED_COLOR : WHITE_COLOR;
            }
        } else {
            for (int i = 0; i < ringLength; i++) {
                ring[i] = GREEN_COLOR;
            }
        }

        ring[0] = WHITE_COLOR;
        ring[ringLength / 2] = WHITE_COLOR;
    }

    private void getThetaIndicatorRing(Color8Bit[] ring, double absRotationError) {
        if (absRotationError > 45.0) {
            for (int i = 0; i < ringLength; i++) {
                ring[i] = i < halfRingLength ? RED_COLOR : BLUE_COLOR;
            }
        } else if (absRotationError > 10.0) {
            for (int i = 0; i < ringLength; i++) {
                ring[i] = i < halfRingLength ? RED_COLOR : ORANGE_COLOR;
            }
        } else if (absRotationError > 1.0) {  // kinda cheesy, but OK.
            for (int i = 0; i < ringLength; i++) {
                ring[i] = i < absRotationError / 1.0 ? RED_COLOR : WHITE_COLOR;
            }
        } else {
            for (int i = 0; i < ringLength; i++) {
                ring[i] = GREEN_COLOR;
            }
        }

        ring[0] = WHITE_COLOR;
        ring[ringLength / 2] = WHITE_COLOR;
    }

    private void setShooterLights() {
        m_ledColorRequested = true;

        switch (m_shooterState) {
            case IDLE -> {
                setRingColor(RING_TOP, WHITE_COLOR);
                setRingColor(RING_MIDDLE_TOP, WHITE_COLOR);
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

    private void setAllianceColor() {
        switch (m_alliance) {
            case RED -> {
                allianceColor = RED_COLOR;
                otherAllianceColor = BLUE_COLOR;
            }
            case BLUE -> {
                allianceColor = BLUE_COLOR;
                otherAllianceColor = RED_COLOR;
            }
            default -> {
                allianceColor = WHITE_COLOR;
                otherAllianceColor = WHITE_COLOR;
            }
        }
        ;
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
        int angleShift = (int) Math.round(n * (r.getDegrees() / 360.0));

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
