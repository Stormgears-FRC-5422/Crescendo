package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogReplaySource;
import org.littletonrobotics.junction.Logger;
import us.hebi.quickbuf.ProtoMessage;

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

    public void recordOutput(String key, byte[] value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, boolean value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, long value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, float value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, double value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, String value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, boolean[] value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, long[] value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, float[] value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, double[] value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, String[] value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, Pose2d... value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, Pose3d... value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, Trajectory value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, SwerveModuleState... value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }

    public static void recordOutput(String key, Mechanism2d value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }
    public static <T extends WPISerializable> void recordOutput(String key, T value) {
        if (useAdvantageKit) {
            Logger.recordOutput(key, value);
        }
    }
}
