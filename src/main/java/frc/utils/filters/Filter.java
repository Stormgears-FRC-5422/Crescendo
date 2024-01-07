package frc.utils.filters;

import java.util.function.DoubleSupplier;

public class Filter {
    protected DoubleSupplier valueSupplier;
    protected double m_value;

    public Filter() {
        valueSupplier = () -> 0.0;
    }

    public Filter(DoubleSupplier valueSupplier) {
        this.valueSupplier = valueSupplier;
    }

    // Override the filter and just start with a specific value
    public double force() {
        return force(valueSupplier.getAsDouble());
    }

    public double force(double newValue) {
        m_value = newValue;
        return m_value;
    }

    public double update() {
        return update(valueSupplier.getAsDouble());
    }

    public double getValue() { return m_value;}

    public double update(double newValue) {
        m_value = newValue;
        return m_value;
    }

}
