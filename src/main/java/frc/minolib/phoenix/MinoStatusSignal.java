package frc.minolib.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Timestamp;

import java.util.function.Function;

/**
 * The MinoStatusSignal class is a wrapper around the StatusSignal class, providing additional
 * functionality such as unit conversion and latency compensation.
 *
 * @param <T> The type of the value held by the StatusSignal.
 */
public class MinoStatusSignal<T> {
    private final StatusSignal<T> statusSignal;
    private final Function<Double, Double> fromNativeUnits;

    public MinoStatusSignal(final StatusSignal<T> statusSignal) {
        this(statusSignal, value -> value);
    }

    public MinoStatusSignal(final StatusSignal<T> statusSignal, final Function<Double, Double> fromNativeUnits) {
        this.statusSignal = statusSignal.clone();
        this.fromNativeUnits = fromNativeUnits;
    }

    public StatusCode setUpdateFrequency(double frequencyHz, double timeoutSeconds) {
        return statusSignal.setUpdateFrequency(frequencyHz, timeoutSeconds);
    }

    public double getAppliedUpdateFrequency() {
        return statusSignal.getAppliedUpdateFrequency();
    }

    public void refresh() {
        statusSignal.refresh();
    }

    public void waitForUpdate(final double timeoutSec) {
        statusSignal.waitForUpdate(timeoutSec);
    }

    public T getRawValue() {
        return statusSignal.getValue();
    }

    public double getUnitConvertedValue() {
        return fromNativeUnits.apply(statusSignal.getValueAsDouble());
    }

    public Timestamp getTimestamp() {
        return statusSignal.getTimestamp();
    }

    public static BaseStatusSignal[] toBaseStatusSignals(final MinoStatusSignal<?>... minoSignals) {
        final BaseStatusSignal[] signals = new BaseStatusSignal[minoSignals.length];
        for (int i = 0; i < minoSignals.length; i++) {
            signals[i] = minoSignals[i].statusSignal;
        }

        return signals;
    }

    public static double getLatencyCompensatedValue(final MinoStatusSignal<?> signal, final MinoStatusSignal<?> signalSlope) {
        return signal.getUnitConvertedValue() + (signalSlope.getUnitConvertedValue() * signal.getTimestamp().getLatency());
    }
}