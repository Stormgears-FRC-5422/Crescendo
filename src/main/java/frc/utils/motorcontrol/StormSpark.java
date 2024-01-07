package frc.utils.motorcontrol;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.*;
import static java.lang.Math.max;
import static java.lang.Math.min;

public class StormSpark extends CANSparkMax {
  private static final double temperatureRampThreshold = SparkMax.TemperatureRampThreshold;
  private static final double temperatureRampLimit = SparkMax.TemperatureRampLimit;
  private final MotorKind motorKind;
  private final double delta;
  private int currentLimit;
  private double scale = 1.0;
  private long count = 0;
  private double groupTemperature;

  public StormSpark(int deviceID, MotorType type, MotorKind kind) {
    super(deviceID, type);

    this.motorKind = kind;

    switch (motorKind) {
      case kNeo:
        currentLimit = (int) SparkMax.CurrentLimit;
        break;
      case k550:
        currentLimit = (int) SparkMax.Neo550CurrentLimit;
        break;
    }

    restoreFactoryDefaults();
    setSmartCurrentLimit(currentLimit);

    delta =
        min(
            temperatureRampLimit - temperatureRampThreshold,
            1.0); // Safety margin - don't want divide by 0!
  }

  public static void check(REVLibError command) {
    if (command != REVLibError.kOk) {
      DriverStation.reportWarning("WARNING: SparkMax action failed!", true);
    }
  }

  public static void check(REVLibError command, String logMessage) {
    if (command != REVLibError.kOk) {
      DriverStation.reportWarning("WARNING: SparkMax action failed: " + logMessage, true);
    }
  }

  @Override
  // We should probably have a way to coordinate this ramp among multiple motors
  // The drive class already does this for its set speed, but the robot will pull left or right if
  // left and right motors are getting different results in this method.
  public void set(double speed) {
    // if the temperature is too high, start ramping down.
    // reduce speed to 0 at temperatureRampLimit
    // start ramping down at temperatureRampThreshold

    double temp = max(getMotorTemperature(), groupTemperature);

    if (temp > temperatureRampThreshold) {
      speed *= max((temperatureRampLimit - temp) / delta, 0.0);
      if (count++ % 100 == 0)
        System.out.println(
            "Id "
                + this.getDeviceId()
                + " safety control - speed: "
                + speed
                + " temperature:"
                + temp);
    }

    super.set(scale * speed);
  }

  // Should be between 0.0 and 1.0 - to account for oddities in the drive train
  // e.g. two different gear ratios
  public void setSpeedScale(double scale) {
    this.scale = MathUtil.clamp(scale, 0, 1.0);
  }

  public void setGroupTemperature(double temperature) {
    groupTemperature = temperature;
  }

  public enum MotorKind {
    kNeo,
    k550
  }
}
