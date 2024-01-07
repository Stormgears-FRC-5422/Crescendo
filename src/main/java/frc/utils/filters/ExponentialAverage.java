package frc.utils.filters;

import frc.utils.configfile.StormProp;

import java.util.function.DoubleSupplier;

public class ExponentialAverage extends Filter {
    // nextValue = alpha * value + (1-alpha) * lastValue
    private int samples = StormProp.getInt("filterSamplesDefault", 50); // start someplace reasonable (50 samples)
    private double alpha;

    public ExponentialAverage() {
        super();
        setAlpha();
    }

    public ExponentialAverage(int samples) {
        super();
        this.samples = samples;
        setAlpha();
    }

    public ExponentialAverage(DoubleSupplier valueSupplier) {
        super(valueSupplier);
        setAlpha();
    }

    public ExponentialAverage(DoubleSupplier valueSupplier, int samples) {
        super(valueSupplier);
        this.samples = samples;
        setAlpha();
    }

    private void setAlpha() {
        // Time constant = - 1 /( ln(1 - alpha) ) samples
        this.alpha = 1.0 - Math.exp(-1.0 / samples);
    }

    @Override
    // Note that there is a built-in doubleSupplier that will call this method
    // if next() is called with no arguments
    public double update(double newValue) {
        // value = alpha * newValue + (1-alpha) * value // simplifies to...
        m_value += alpha * (newValue - m_value);
        return m_value;
    }

}
