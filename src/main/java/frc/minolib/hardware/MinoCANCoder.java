package frc.minolib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.MinoStatusSignal;
import frc.minolib.phoenix.PhoenixUtility;

public class MinoCANCoder implements AutoCloseable {
    private static final double kCANTimeoutSeconds = 0.1; // s
    
    private final String name;
    private final CANcoder cancoder;
    private final CANcoderSimState simulationState;
    private final MechanismRatio gearRatio;

    private final MinoStatusSignal<Integer> faultFieldSignal;
    private final MinoStatusSignal<Integer> stickyFaultFieldSignal;
    private final MinoStatusSignal<MagnetHealthValue> magnetHealthSignal;
    private final MinoStatusSignal<Voltage> supplyVoltageSignal;

    private final MinoStatusSignal<Angle> positionSignal;
    private final MinoStatusSignal<Angle> absolutePositionSignal;
    private final MinoStatusSignal<AngularVelocity> velocitySignal;

    private final Alert unlicensedFeatureAlert;

    public static class MinoCANCoderConfiguration {
        private double magnetOffset = 0.0;
        private double absoluteSensorDiscontinuityPoint = 1.0;
        private boolean isInverted = false;

        public MinoCANCoderConfiguration withMagnetOffset(double magnetOffset) {
            this.magnetOffset = magnetOffset;
            return this;
        }

        public MinoCANCoderConfiguration withAbsoluteSensorDiscontinuity(double point) {
            this.absoluteSensorDiscontinuityPoint = point;
            return this;
        }

        public MinoCANCoderConfiguration setInverted(boolean isInverted) {
            this.isInverted = isInverted;
            return this;
        }

        CANcoderConfiguration toCANCoderConfiguration() {
            final CANcoderConfiguration configuration = new CANcoderConfiguration();

            configuration.MagnetSensor.MagnetOffset = magnetOffset;
            configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteSensorDiscontinuityPoint;
            configuration.MagnetSensor.SensorDirection = isInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

            return configuration;
        }
    }

    public static MinoCANCoderConfiguration makeDefaultConfig() {
        return new MinoCANCoderConfiguration();
    }

    public MinoCANCoder(final MinoCANDevice canDevice, final MechanismRatio gearRatio, final MinoCANCoderConfiguration configuration) {
        this.name = "CANCoder " + canDevice.toString();
        this.cancoder = new CANcoder(canDevice.deviceNumber, canDevice.CANbusName.toString());
        this.simulationState = cancoder.getSimState();
        this.gearRatio = gearRatio;
  
        faultFieldSignal = new MinoStatusSignal<>(cancoder.getFaultField());
        stickyFaultFieldSignal = new MinoStatusSignal<>(cancoder.getStickyFaultField());
        magnetHealthSignal = new MinoStatusSignal<>(cancoder.getMagnetHealth());
        supplyVoltageSignal= new MinoStatusSignal<>(cancoder.getSupplyVoltage());

        positionSignal = new MinoStatusSignal<>(
            cancoder.getPosition(),
            rotations -> gearRatio.sensorRadiansToMechanismPosition(rotations * 2.0 * Math.PI)
        );

        absolutePositionSignal = new MinoStatusSignal<>(
            cancoder.getAbsolutePosition(),
            rotations -> gearRatio.sensorRadiansToMechanismPosition(rotations * 2.0 * Math.PI)
        );

        velocitySignal = new MinoStatusSignal<>(
            cancoder.getVelocity(),
            rotationsPerSecond -> gearRatio.sensorRadiansToMechanismPosition(rotationsPerSecond * 2.0 * Math.PI)
        );

        unlicensedFeatureAlert = new Alert(name + " is using an unlicensed feature. Device behavior may be restricted.", AlertType.kError);
        
        cancoder.hasResetOccurred();
        cancoder.clearStickyFaults();

        applyConfiguration(configuration);
    }

    private boolean applyConfiguration(final MinoCANCoderConfiguration configuration) {
        boolean allSuccess = true;

        final CANcoderConfiguration config = configuration.toCANCoderConfiguration();
        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> cancoder.getConfigurator().apply(config, kCANTimeoutSeconds),
            () -> {
                CANcoderConfiguration readConfig = new CANcoderConfiguration();
                cancoder.getConfigurator().refresh(readConfig, kCANTimeoutSeconds);
                return PhoenixUtility.CANcoderConfigsEqual(config, readConfig);
            },
            name + ": applyConfiguration"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> cancoder.optimizeBusUtilization(0.0, kCANTimeoutSeconds),
            name + ": optimizeBusUtilization"
        );

        final StatusSignal<Boolean> unlicensedSignal = cancoder.getStickyFault_UnlicensedFeatureInUse();
        unlicensedSignal.waitForUpdate(kCANTimeoutSeconds);

        if (unlicensedSignal.getValue()) {
            final String message = name + " has an unlicensed feature in use. Device behavior may be restricted.";
            unlicensedFeatureAlert.set(true);
            DriverStation.reportError(message, false);
        }

        return allSuccess;
    }

    public String getName() {
        return name;
    }

    public CANcoderSimState getSimulatedState() {
        return simulationState;
    }

    public MinoStatusSignal<Integer> getFaultFieldSignal() {
        return faultFieldSignal; 
    }

    public MinoStatusSignal<Integer> getStickyFaultFieldSignal() { 
        return stickyFaultFieldSignal; 
    }

    public MinoStatusSignal<MagnetHealthValue> getMagnetHealthSignal() { 
        return magnetHealthSignal; 
    }

    public MinoStatusSignal<Voltage> getSupplyVoltageSignal() { 
        return supplyVoltageSignal; 
    }

    public MinoStatusSignal<Angle> getPositionSignal() { 
        return positionSignal; 
    }

    public MinoStatusSignal<Angle> getAbsolutePositionSignal() { 
        return absolutePositionSignal; 
    }

    public MinoStatusSignal<AngularVelocity> getVelocitySignal() { 
        return velocitySignal; 
    }

    public BaseStatusSignal[] getAllImportantSignals() {
        return MinoStatusSignal.toBaseStatusSignals(
            faultFieldSignal,
            stickyFaultFieldSignal,
            magnetHealthSignal,
            supplyVoltageSignal,
            positionSignal,
            absolutePositionSignal,
            velocitySignal
        );
    }

    public void zero() {
        setPosition(0.0);
    }

    public void setPosition(final double mechanismPosition) {
        cancoder.setPosition(gearRatio.mechanismPositionToSensorRadians(mechanismPosition) / (2.0 * Math.PI));
    }

    public void setSimSensorVelocity(final double mechanismVelocity, final double dt) {
        final double rotationsPerSecond = gearRatio.mechanismPositionToSensorRadians(mechanismVelocity) / (2.0 * Math.PI);

        simulationState.setVelocity(rotationsPerSecond);
        simulationState.addPosition(rotationsPerSecond * dt);
    }

    @Override
    public void close() {
        cancoder.close();
    }
}
