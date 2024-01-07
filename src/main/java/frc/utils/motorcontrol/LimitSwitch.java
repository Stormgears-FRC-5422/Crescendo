package frc.utils.motorcontrol;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch extends DigitalInput {

    private boolean enabled;

    public LimitSwitch(int channel, boolean enabled) {
        super(channel);
        this.enabled = enabled;
    }

    public void disable() {
        enabled = false;
    }

    public void enable() {
        enabled = true;
    }

    @Override
    public boolean get() {
        return (!super.get()) && enabled;
    }

  public boolean getAbsoluteLimit() {
    return !super.get();
  }
}
