package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
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
    private boolean m_allianceIsBlue;

    public static final Color8Bit RED_COLOR = new Color8Bit(200, 0, 0);
    public static final Color8Bit GREEN_COLOR = new Color8Bit(0, 200, 0);
    public static final Color8Bit BLUE_COLOR = new Color8Bit(0, 0, 200);
    public static final Color8Bit ORANGE_COLOR = new Color8Bit(200, 129, 0);
    public static final Color8Bit WHITE_COLOR = new Color8Bit(200, 200, 200);
    public static final Color8Bit NO_COLOR = new Color8Bit(0, 0, 0);

    private Segment RING_TOP;
    private Segment RING_BOTTOM;
    private Segment RING_MIDDLE_TOP;
    private Segment RING_MIDDLE_BOTTOM;

    private LEDLightStrip m_ledLightStrip;
    boolean m_ledColorRequested;

    public StatusLights() {
        m_robotState = RobotState.getInstance();
        m_shooterState = m_robotState.getShooterState();
        m_allianceIsBlue = m_robotState.isAllianceBlue();

        initializeLights();
        setShooterLights();
        setAllianceLights();
//        setDriveLights();
    }

    public void periodic() {
        // don't bother setting the lights again unless they have changed.
        if (m_shooterState != m_robotState.getShooterState()) {
            m_shooterState = m_robotState.getShooterState();
            setShooterLights();
        }

        if ( m_allianceIsBlue == m_robotState.isAllianceBlue()) {
            m_allianceIsBlue = m_robotState.isAllianceBlue();
            setAllianceLights();
        }

        if (m_ledColorRequested) {
            m_ledLightStrip.setLEDData();
        }
    }

    public void initializeLights() {
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
    }

    private void setShooterLights() {
        m_ledColorRequested = true;

        switch (m_shooterState) {
            case IDLE -> {
                m_ledLightStrip.setLEDColor(RING_TOP.number, WHITE_COLOR);
                m_ledLightStrip.setLEDColor(RING_MIDDLE_TOP.number, WHITE_COLOR);
                m_ledLightStrip.setLEDColor(RING_MIDDLE_BOTTOM.number, WHITE_COLOR);
            }
            case STAGED_FOR_SHOOTING -> {
                m_ledLightStrip.setLEDColor(RING_TOP.number, ORANGE_COLOR);
                m_ledLightStrip.setLEDColor(RING_MIDDLE_TOP.number, ORANGE_COLOR);
                m_ledLightStrip.setLEDColor(RING_MIDDLE_BOTTOM.number, ORANGE_COLOR);
            }
            case GROUND_PICKUP -> {
                m_ledLightStrip.setLEDColor(RING_TOP.number, WHITE_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_TOP.number, WHITE_COLOR, ORANGE_COLOR);
                m_ledLightStrip.setLEDColor(RING_MIDDLE_BOTTOM.number, ORANGE_COLOR);
            }
            case SOURCE_PICKUP_1, SOURCE_PICKUP_2 -> {
                m_ledLightStrip.setLEDColor(RING_TOP.number, ORANGE_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_TOP.number, WHITE_COLOR, ORANGE_COLOR);
                m_ledLightStrip.setLEDColor(RING_MIDDLE_BOTTOM.number, WHITE_COLOR);
            }
            case SPEAKER_SHOOTING -> {
                m_ledLightStrip.setLEDColor(RING_TOP.number, GREEN_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_TOP.number, WHITE_COLOR, GREEN_COLOR);
                m_ledLightStrip.setLEDColor(RING_MIDDLE_BOTTOM.number, WHITE_COLOR);
            }
            case AMP_SHOOTING -> {
                m_ledLightStrip.setLEDColor(RING_TOP.number, WHITE_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_TOP.number, WHITE_COLOR, GREEN_COLOR);
                m_ledLightStrip.setLEDColor(RING_MIDDLE_BOTTOM.number, GREEN_COLOR);
            }
            case DIAGNOSTIC -> {
                m_ledLightStrip.setAlternatingLEDColor(RING_TOP.number, WHITE_COLOR, RED_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_TOP.number, RED_COLOR, BLUE_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_BOTTOM.number, BLUE_COLOR, WHITE_COLOR);
            }
            case OUTTAKE -> {
                m_ledLightStrip.setAlternatingLEDColor(RING_TOP.number, ORANGE_COLOR, GREEN_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_TOP.number, GREEN_COLOR, ORANGE_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_BOTTOM.number, ORANGE_COLOR, GREEN_COLOR);
            }
            case BAD -> {
                m_ledLightStrip.setAlternatingLEDColor(RING_TOP.number, WHITE_COLOR, ORANGE_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_TOP.number, ORANGE_COLOR, WHITE_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_BOTTOM.number, WHITE_COLOR, ORANGE_COLOR);
            }
            default -> {
                m_ledLightStrip.setAlternatingLEDColor(RING_TOP.number, WHITE_COLOR, NO_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_TOP.number, NO_COLOR, WHITE_COLOR);
                m_ledLightStrip.setAlternatingLEDColor(RING_MIDDLE_BOTTOM.number, WHITE_COLOR, NO_COLOR);
            }
        }
    }

    private void setAllianceLights() {
        m_ledColorRequested = true;
        m_ledLightStrip.setLEDColor(RING_BOTTOM.number, m_allianceIsBlue ? BLUE_COLOR : RED_COLOR);
    }

}
