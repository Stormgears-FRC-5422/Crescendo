package frc.utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogReplaySource;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Toggles.useAdvantageKit;

public class LoggerWrapper {
    public static void start() {
        if (useAdvantageKit) {
            Logger.start();
        } else {

        }
    }

    public static void recordMetadata(String key, String value) {
        if (useAdvantageKit) {
            Logger.recordMetadata(key, value);
        } else {

        }
    }

    public static void addDataReceiver(LogDataReceiver dataReceiver) {
        if (useAdvantageKit) {
            Logger.addDataReceiver(dataReceiver);
        } else {

        }
    }

    public static void enablePowerDistributionLogging() {
        if (useAdvantageKit) {
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
        }
    }

    public static void setReplaySource(LogReplaySource replaySource) {
        if (useAdvantageKit) {
            Logger.setReplaySource(replaySource);
        }
    }
}