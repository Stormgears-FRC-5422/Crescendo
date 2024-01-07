package frc.utils.filters;

import frc.utils.configfile.StormProp;

import java.util.function.DoubleSupplier;

public class MovingAverage extends Filter {
    private int samples = StormProp.getInt("filterSamplesDefault", 50); // start someplace reasonable

    public MovingAverage(int samples) {
        super();
        this.samples = samples;
    }

    public MovingAverage(DoubleSupplier valueSupplier, int samples) {
        super(valueSupplier);
        this.samples = samples;
    }

    @Override
    // Note that there is a built-in doubleSupplier that will call this method
    // if next() is called with no arguments
    public double update(double newValue) {
        // lastValue = ( ((samples - 1) * lastValue) + value) / samples; // simplifies to
        m_value += (newValue - m_value) / samples;
        return m_value;
    }

}
