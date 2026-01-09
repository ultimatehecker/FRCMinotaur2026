package frc.minolib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.minolib.io.AbsoluteEncoderInputsAutoLogged;
import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.MinoStatusSignal;
import frc.minolib.phoenix.PhoenixEncoder;
import frc.minolib.phoenix.PhoenixUtility;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

/**
 * The MinoCANCoder class implements the PhoenixIO and MinoAbsoluteEncoder interfaces to provide
 * functionality for interacting with a CANcoder device.
 *
 * <p>This class handles the configuration, input updates, and simulation state of the CANcoder. It
 * also provides methods to get and set the position and velocity of the encoder.
 *
 * <p>Constructor:
 *
 * <ul>
 *   <li>{@link #MinoCANCoder(CANDeviceID, MechanismRatio)}: Initializes a new instance of the
 *       MinoCANCoder class with the specified CAN device ID and mechanism ratio.
 * </ul>
 *
 * <p>Public Methods:
 *
 * <ul>
 *   <li>{@link #setConfiguration()}: Configures the CANcoder and sets update frequencies for
 *       various signals.
 *   <li>{@link #updateInputs()}: Updates the input signals from the CANcoder.
 *   <li>{@link #waitForInputs(double)}: Waits for input signals to be updated within the specified
 *       timeout period.
 *   <li>{@link #zero()}: Sets the position of the CANcoder to zero.
 *   <li>{@link #setPosition(double)}: Sets the position of the CANcoder to the specified value.
 *   <li>{@link #getPosition()}: Gets the current position of the CANcoder.
 *   <li>{@link #getAbsPosition()}: Gets the absolute position of the CANcoder.
 *   <li>{@link #getVelocity()}: Gets the current velocity of the CANcoder.
 *   <li>{@link #setSimSensorVelocity(double, double)}: Sets the simulated sensor velocity and
 *       updates the simulation state.
 * </ul>
 *
 * <p>Private Methods:
 *
 * <ul>
 *   <li>{@link #toNativeSensorPosition(double)}: Converts a position value to native sensor units.
 *   <li>{@link #fromNativeSensorPosition(double)}: Converts a native sensor position value to
 *       mechanism units.
 *   <li>{@link #toNativeSensorVelocity(double)}: Converts a velocity value to native sensor units.
 *   <li>{@link #fromNativeSensorVelocity(double)}: Converts a native sensor velocity value to
 *       mechanism units.
 * </ul>
 *
 * <p>Inner Classes:
 *
 * <ul>
 *   <li>{@link CANCoderInputs}: A class representing the input signals from the CANcoder.
 * </ul>
 *
 * <p>Fields:
 *
 * <ul>
 *   <li>{@code kCANTimeoutS}: The timeout value for CAN operations, in seconds.
 *   <li>{@code name}: The name of the CANcoder instance.
 *   <li>{@code loggingName}: The logging name for the CANcoder instance.
 *   <li>{@code cancoder}: The CANcoder device instance.
 *   <li>{@code simulationState}: The simulation state of the CANcoder.
 *   <li>{@code gearRatio}: The mechanism ratio for converting between sensor and mechanism units.
 *   <li>{@code faultFieldSignal}: The signal representing the fault field of the CANcoder.
 *   <li>{@code stickyFaultFieldSignal}: The signal representing the sticky fault field of the
 *       CANcoder.
 *   <li>{@code positionSignal}: The signal representing the position of the CANcoder.
 *   <li>{@code absolutePositionSignal}: The signal representing the absolute position of the
 *       CANcoder.
 *   <li>{@code velocitySignal}: The signal representing the velocity of the CANcoder.
 *   <li>{@code allSignals}: An array of all status signals for the CANcoder.
 *   <li>{@code inputs}: An instance of the CANCoderInputs class representing the current input
 *       signals.
 * </ul>
 */

public class MinoCANCoder implements AutoCloseable, PhoenixEncoder {
    private static final double kCANTimeoutS = 0.1; // s
    private final String name;
    private final String loggingName;
    private final CANcoder cancoder;
    private final CANcoderSimState simulationState;
    private final MechanismRatio gearRatio;
    private final MinoCANCoderConfiguration configuration;

    private final MinoStatusSignal<Integer> faultFieldSignal;
    private final MinoStatusSignal<Integer> stickyFaultFieldSignal;
    private final MinoStatusSignal<Angle> positionSignal;
    private final MinoStatusSignal<Angle> absolutePositionSignal;
    private final MinoStatusSignal<AngularVelocity> velocitySignal;
    private final BaseStatusSignal[] allSignals;

    private final AbsoluteEncoderInputsAutoLogged inputs = new AbsoluteEncoderInputsAutoLogged();

    public final Alert disconnectedAlert;
    public final Alert unoptimizedMagnetAlert; // TODO: Fix the uptmoized magnet to split into unuable and unoptimized
    public final Alert unusableMagnetAlert;
    public final Alert supplyVoltageAlert;

    public static class MinoCANCoderConfiguration {
        private double magnetOffset = 0.0;
        private double absoluteSensorDiscontinuityPoint = 1.0;
        private boolean isInverted;

        public MinoCANCoderConfiguration withMagnetOffset(double magnetOffset) {
            this.magnetOffset = magnetOffset;
            return this;
        }

        public MinoCANCoderConfiguration setInverted(boolean isInverted) {
            this.isInverted = isInverted;
            return this;
        }

        public MinoCANCoderConfiguration withAbsoluteSensorDiscontinuity(double absoluteSensorDiscontinuityPoint) {
            this.absoluteSensorDiscontinuityPoint = absoluteSensorDiscontinuityPoint;
            return this;
        }

        public CANcoderConfiguration toCANCoderConfiguration(final Function<Double, Double> toNativeSensorPosition, final Function<Double, Double> toNativeSensorVelocity) {
            final CANcoderConfiguration configuration = new CANcoderConfiguration();
            configuration.MagnetSensor.MagnetOffset = magnetOffset;
            configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteSensorDiscontinuityPoint;
            configuration.MagnetSensor.SensorDirection = isInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

            return configuration;
        }
    }

    public MinoCANCoder(final CANDeviceID canID, final MechanismRatio ratio, final MinoCANCoderConfiguration configuration) {
        name = "CANCoder " + canID.toString();
        loggingName = "Inputs/" + name;
        cancoder = new CANcoder(canID.deviceNumber, canID.CANbusName);
        simulationState = cancoder.getSimState();
        gearRatio = ratio;
        this.configuration = configuration;

        disconnectedAlert = new Alert("CANcoder [" + canID.toString() + "] is currently disconnected. Mechanism may not function as wanted", AlertType.kError);
        unoptimizedMagnetAlert = new Alert("CANcoder [" + canID.toString() + "] magnet is unoptimal. Reduced reading accuracy is guarenteed", AlertType.kWarning);
        unusableMagnetAlert = new Alert("CANcoder [" + canID.toString() + "] magnet is unusable. Mechanism may not function as wanted", AlertType.kError);
        supplyVoltageAlert = new Alert("CANcoder [" + canID.toString() + "] is underpowered. Device might be permentally damaged", AlertType.kWarning);

        faultFieldSignal = new MinoStatusSignal<>(cancoder.getFaultField());
        stickyFaultFieldSignal = new MinoStatusSignal<>(cancoder.getStickyFaultField());
        positionSignal = new MinoStatusSignal<>(cancoder.getPosition(), this::fromNativeSensorPosition);
        absolutePositionSignal = new MinoStatusSignal<>(cancoder.getAbsolutePosition(), this::fromNativeSensorPosition);
        velocitySignal = new MinoStatusSignal<>(cancoder.getVelocity(), this::fromNativeSensorVelocity);
        allSignals = MinoStatusSignal.toBaseStatusSignals(
            faultFieldSignal,
            stickyFaultFieldSignal,
            positionSignal,
            absolutePositionSignal,
            velocitySignal
        );

        // Clear reset flag and sticky faults.
        cancoder.hasResetOccurred();
        cancoder.clearStickyFaults();

        Logger.recordOutput("Configuration/" + name, setConfiguration());
    }

    public boolean setConfiguration() {
        boolean allSuccess = true;

        // Set configuration.
        final CANcoderConfiguration config = configuration.toCANCoderConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity);

        allSuccess &= PhoenixUtility.retryUntilSuccess(() -> cancoder.getConfigurator().apply(config, kCANTimeoutS), () -> {
            CANcoderConfiguration readConfig = new CANcoderConfiguration();
            cancoder.getConfigurator().refresh(readConfig, kCANTimeoutS);
            return PhoenixUtility.CANcoderConfigsEqual(config, readConfig);
        }, name + ": applyConfiguration");

        // Set update frequencies.
        final double kFaultUpdateFrequency = 4.0; // Hz
        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> faultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
            () -> faultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
            name + ": faultFieldSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> stickyFaultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
            () -> stickyFaultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
            name + ": stickyFaultFieldSignal.setUpdateFrequency()"
        );

        final double kUpdateFrequency = 10.0; // Hz
        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> positionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> positionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": positionSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> absolutePositionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> absolutePositionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": absolutePositionSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> velocitySignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> velocitySignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": velocitySignal.setUpdateFrequency()"
        );

        // Disable all signals that have not been explicitly defined.
        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> cancoder.optimizeBusUtilization(0.0, kCANTimeoutS),
            name + ": optimizeBusUtilization"
        );

        // Block until we get valid signals.
        allSuccess &= BaseStatusSignal.waitForAll(kCANTimeoutS, allSignals).isOK();

        // Check if unlicensed.
        allSuccess &= !cancoder.getStickyFault_UnlicensedFeatureInUse().getValue();

        return allSuccess;
    }

    public static MinoCANCoderConfiguration makeDefaultConfig() {
        return new MinoCANCoderConfiguration();
    }

    public void close() {
        cancoder.close();
    }

    public StatusCode updateInputs() {
        disconnectedAlert.set(inputs.isEncoderConnected);
        unoptimizedMagnetAlert.set(cancoder.getMagnetHealth().getValue().equals(MagnetHealthValue.Magnet_Orange));
        unusableMagnetAlert.set(!cancoder.getMagnetHealth().getValue().equals(MagnetHealthValue.Magnet_Green) && unoptimizedMagnetAlert.get());
        supplyVoltageAlert.set(cancoder.getSupplyVoltage().getValue().in(Volts) < 11.5);
        
        return waitForInputs(0.0);
    }

    public StatusCode waitForInputs(final double timeoutSeconds) {
        inputs.isEncoderConnected = BaseStatusSignal.isAllGood(allSignals);
        inputs.status = BaseStatusSignal.waitForAll(timeoutSeconds, allSignals);

        inputs.faultField = faultFieldSignal.getRawValue();
        inputs.stickyFaultField = stickyFaultFieldSignal.getRawValue();
        inputs.position = positionSignal.getUnitConvertedValue();
        inputs.absolutePosition = absolutePositionSignal.getUnitConvertedValue();
        inputs.velocity = velocitySignal.getUnitConvertedValue();

        Logger.processInputs(loggingName, inputs);

        return inputs.status;
    }

    public void zero() {
        setPosition(0.0);
    }

    public void setPosition(final double pos) {
        cancoder.setPosition(toNativeSensorPosition(pos));
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getAbsolutePosition() {
        return inputs.absolutePosition;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    private double toNativeSensorPosition(final double pos) {
        // Native units are in rotations.
        return gearRatio.mechanismPositionToSensorRadians(pos) / (2.0 * Math.PI);
    }

    private double fromNativeSensorPosition(final double pos) {
        return pos / toNativeSensorPosition(1.0);
    }

    private double toNativeSensorVelocity(final double vel) {
        return toNativeSensorPosition(vel);
    }

    private double fromNativeSensorVelocity(final double vel) {
        return vel / toNativeSensorVelocity(1.0);
    }

    public CANcoderSimState getSimulatedState() {
        return simulationState;
    }

    public void setSimSensorVelocity(final double vel, final double dt) {
        final double rotationsPerSecond = toNativeSensorVelocity(vel);
        simulationState.setVelocity(rotationsPerSecond);
        simulationState.addPosition(rotationsPerSecond * dt);
    }
}
