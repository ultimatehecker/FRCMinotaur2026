package frc.minolib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.MinoStatusSignal;
import frc.minolib.phoenix.PIDConfiguration;
import frc.minolib.phoenix.PhoenixUtility;
import frc.minolib.io.MotorInputsAutoLogged;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class MinoTalonFX implements AutoCloseable {
    private static final double kCANTimeoutSeconds = 0.1; 
    
    private final String name;
    private final TalonFX controller;
    private final TalonFXSimState simulationState;
    private final MechanismRatio gearRatio;
    private final MinoTalonFXConfiguration configuration;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
    private final MotionMagicExpoVoltage motionMagicExpoControl = new MotionMagicExpoVoltage(0);

    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityControlFOC = new VelocityTorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionControlFOC = new PositionTorqueCurrentFOC(0);
    private final MotionMagicTorqueCurrentFOC motionMagicFOCControl = new MotionMagicTorqueCurrentFOC(0);
    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoFOCControl = new MotionMagicExpoTorqueCurrentFOC(0);
    private final DynamicMotionMagicVoltage dynamicMotionMagicControl = new DynamicMotionMagicVoltage(0, 0, 0);

    private final MinoStatusSignal<Integer> faultFieldSignal;
    private final MinoStatusSignal<Integer> stickyFaultFieldSignal;
    private final MinoStatusSignal<Double> percentOutputSignal;
    private final MinoStatusSignal<Angle> sensorPositionSignal;
    private final MinoStatusSignal<AngularVelocity> sensorVelocitySignal;
    private final MinoStatusSignal<AngularAcceleration> sensorAccelerationSignal;
    private final MinoStatusSignal<Double> closedLoopReferenceSignal;
    private final MinoStatusSignal<Double> closedLoopReferenceSlopeSignal;
    private final MinoStatusSignal<Current> supplyCurrentSignal;
    private final MinoStatusSignal<Current> statorCurrentSignal;
    private final MinoStatusSignal<Current> torqueCurrentSignal;
    private final MinoStatusSignal<Temperature> temperatureSignal;

    private final Alert unlicensedFeatureAlert;

    public static class MinoTalonFXConfiguration {
        private NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        private boolean kInverted = false;

        private double kSupplyCurrentLimit = 40.0; // A
        private double kStatorCurrentLimit = 40.0; // A

        private boolean kForwardSoftLimitEnabled = false;
        private double kForwardSoftLimit = 0.0; // In MechanismRatio units
        private boolean kReverseSoftLimitEnabled = false;
        private double kReverseSoftLimit = 0.0; // In MechanismRatio units

        private PIDConfiguration kSlot0Configuration = new PIDConfiguration();
        private PIDConfiguration kSlot1Configuration = new PIDConfiguration();
        private PIDConfiguration kSlot2Configuration = new PIDConfiguration();

        private double kMotionMagicCruiseVelocity = 0.0;
        private double kMotionMagicAcceleration = 0.0;
        private double kMotionMagicJerk = 0.0; 
        private double kMotionMagicExpokA = 0.0;
        private double kMotionMagicExpokV = 0.0;

        private CANDeviceID feedbackDeviceID = new CANDeviceID(0);
        private FeedbackSensorSourceValue feedbackSource = FeedbackSensorSourceValue.RemoteCANcoder;
        private boolean continuousWrappedEnabled = false;

        public MinoTalonFXConfiguration setBrakeMode() {
            kNeutralMode = NeutralModeValue.Brake;
            return this;
        }

        public MinoTalonFXConfiguration setInverted(final boolean inverted) {
            kInverted = inverted;
            return this;
        }

        public MinoTalonFXConfiguration setStatorCurrentLimit(final double amperes) {
            kStatorCurrentLimit = amperes;
            return this;
        }

        public MinoTalonFXConfiguration setSupplyCurrentLimit(final double amperes) {
            kSupplyCurrentLimit = amperes;
            return this;
        }

        public MinoTalonFXConfiguration setForwardSoftLimit(final double position) {
            kForwardSoftLimitEnabled = true;
            kForwardSoftLimit = position;
            return this;
        }

        public MinoTalonFXConfiguration setReverseSoftLimit(final double position) {
            kReverseSoftLimitEnabled = true;
            kReverseSoftLimit = position;
            return this;
        }

        public MinoTalonFXConfiguration setPIDConfiguration(final int slot, final PIDConfiguration configuration) {
            switch (slot) {
                case 0:
                    kSlot0Configuration = configuration;
                    break;
                case 1:
                    kSlot1Configuration = configuration;
                    break;
                case 2:
                    kSlot2Configuration = configuration;
                    break;
                default:
                    throw new RuntimeException("Invalid PID slot " + slot);
            }

            return this;
        }

        public MinoTalonFXConfiguration setMotionMagicConfig(final double cruiseVelocity, final double acceleration, final double jerk) {
            kMotionMagicCruiseVelocity = cruiseVelocity;
            kMotionMagicAcceleration = acceleration;
            kMotionMagicJerk = jerk;
            return this;
        }

        public MinoTalonFXConfiguration setMotionMagicExpoConfig(final double kA, final double kV) {
            kMotionMagicExpokA = kA;
            kMotionMagicExpokV = kV;
            return this;
        }

        public MinoTalonFXConfiguration setSteerFeedback(final int deviceNumber, final FeedbackSensorSourceValue feedbackSource) {
            feedbackDeviceID.newDeviceID(deviceNumber);
            this.feedbackSource = feedbackSource;
            return this;
        }

        public MinoTalonFXConfiguration enableContinuousWrapping(boolean enabled) {
            continuousWrappedEnabled = enabled;
            return this;
        }

        public TalonFXConfiguration toTalonFXConfiguration(final Function<Double, Double> toNativeSensorPosition, final Function<Double, Double> toNativeSensorVelocity) {
            final TalonFXConfiguration configuration = new TalonFXConfiguration();
            configuration.MotorOutput.NeutralMode = kNeutralMode;
            configuration.MotorOutput.Inverted = kInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            configuration.MotorOutput.DutyCycleNeutralDeadband = 0.0;

            configuration.CurrentLimits.StatorCurrentLimitEnable = true;
            configuration.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;

            configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
            configuration.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
            configuration.CurrentLimits.SupplyCurrentLowerLimit = kSupplyCurrentLimit;
            configuration.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            configuration.TorqueCurrent.PeakForwardTorqueCurrent = kStatorCurrentLimit;
            configuration.TorqueCurrent.PeakReverseTorqueCurrent = -kStatorCurrentLimit;
            configuration.TorqueCurrent.TorqueNeutralDeadband = 0.0;

            configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = kForwardSoftLimitEnabled;
            configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = toNativeSensorPosition.apply(kForwardSoftLimit);
            configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = kReverseSoftLimitEnabled;
            configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = toNativeSensorPosition.apply(kReverseSoftLimit);

            configuration.Feedback.FeedbackSensorSource = feedbackSource;
            configuration.Feedback.FeedbackRemoteSensorID = feedbackDeviceID.deviceNumber;

            configuration.Voltage.SupplyVoltageTimeConstant = 0.0;
            configuration.Voltage.PeakForwardVoltage = 16.0;
            configuration.Voltage.PeakReverseVoltage = -16.0;

            configuration.Slot0 = kSlot0Configuration.fillCTRE(new Slot0Configs());
            configuration.Slot1 = kSlot1Configuration.fillCTRE(new Slot1Configs());
            configuration.Slot2 = kSlot2Configuration.fillCTRE(new Slot2Configs());

            configuration.MotionMagic.MotionMagicCruiseVelocity = toNativeSensorVelocity.apply(kMotionMagicCruiseVelocity);
            configuration.MotionMagic.MotionMagicAcceleration = toNativeSensorVelocity.apply(kMotionMagicAcceleration);
            configuration.MotionMagic.MotionMagicJerk = toNativeSensorVelocity.apply(kMotionMagicJerk);
            configuration.MotionMagic.MotionMagicExpo_kA = kMotionMagicExpokA;
            configuration.MotionMagic.MotionMagicExpo_kV = kMotionMagicExpokV;

            return configuration;
        }
    }

    public static MinoTalonFXConfiguration makeDefaultConfig() {
        return new MinoTalonFXConfiguration();
    }

    public MinoTalonFX(final CANDeviceID canID, final MinoTalonFX leader, final MinoTalonFXConfiguration configuration) {
        this(canID, leader.getMechanismRatio(), configuration);
        controller.setControl(new StrictFollower(leader.getDeviceID()));
    }

    public MinoTalonFX(final CANDeviceID canID, final MechanismRatio gearRatio, final MinoTalonFXConfiguration configuration) {
        name = "TalonFX " + canID.toString();
        controller = new TalonFX(canID.deviceNumber, canID.CANbusName);
        simulationState = controller.getSimState();
        this.gearRatio = gearRatio;
        this.configuration = configuration;

        faultFieldSignal = new MinoStatusSignal<>(controller.getFaultField());
        stickyFaultFieldSignal = new MinoStatusSignal<>(controller.getStickyFaultField());
        percentOutputSignal = new MinoStatusSignal<>(controller.getDutyCycle());
        supplyCurrentSignal = new MinoStatusSignal<>(controller.getSupplyCurrent());
        statorCurrentSignal = new MinoStatusSignal<>(controller.getStatorCurrent());
        torqueCurrentSignal = new MinoStatusSignal<>(controller.getTorqueCurrent());
        sensorPositionSignal = new MinoStatusSignal<>(controller.getPosition(), this::fromNativeSensorPosition);
        sensorVelocitySignal = new MinoStatusSignal<>(controller.getVelocity(), this::fromNativeSensorVelocity);
        sensorAccelerationSignal = new MinoStatusSignal<>(controller.getAcceleration(), this::fromNativeSensorAcceleration);
        closedLoopReferenceSignal = new MinoStatusSignal<>(controller.getClosedLoopReference(), this::fromNativeSensorPosition);
        closedLoopReferenceSlopeSignal = new MinoStatusSignal<>(controller.getClosedLoopReferenceSlope(), this::fromNativeSensorVelocity);
        temperatureSignal = new MinoStatusSignal<>(controller.getDeviceTemp());

        unlicensedFeatureAlert = new Alert(name + " is using an unlicensed feature. Device behavior may be restricted.", AlertType.kError);

        controller.hasResetOccurred();
        controller.clearStickyFaults();

        applyConfiguration();
    }

    public boolean applyConfiguration() {
        boolean allSuccess = true;

        final TalonFXConfiguration config = configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity);
        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> controller.getConfigurator().apply(config, kCANTimeoutSeconds),
            () -> {
                TalonFXConfiguration readConfig = new TalonFXConfiguration();
                controller.getConfigurator().refresh(readConfig, kCANTimeoutSeconds);
                return PhoenixUtility.TalonFXConfigsEqual(config, readConfig);
            },
            name + ": applyConfiguration"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> controller.optimizeBusUtilization(0.0, kCANTimeoutSeconds),
            name + ": optimizeBusUtilization"
        );

        final StatusSignal<Boolean> unlicensedSignal = controller.getStickyFault_UnlicensedFeatureInUse();
        unlicensedSignal.waitForUpdate(kCANTimeoutSeconds);

        if (unlicensedSignal.getValue()) {
            final String message = name + " has an unlicensed feature in use. Device behavior may be restricted.";
            unlicensedFeatureAlert.set(true);
            DriverStation.reportError(message, false);
        }

        return allSuccess;
    }

    public MinoStatusSignal<Integer> getFaultFieldSignal() {
      return faultFieldSignal;
    }

    public MinoStatusSignal<Integer> getStickyFaultFieldSignal() {
        return stickyFaultFieldSignal;
    }

    public MinoStatusSignal<Angle> getRollSignal() {
        return sensorPositionSignal;
    }

    public StatusCode setBrakeMode(final boolean on) {
        configuration.kNeutralMode = on ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        StatusCode ok = controller.getConfigurator().apply(configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity).MotorOutput);

        return ok;
    }

    public void setStatorCurrentLimit(final double amps) {
        configuration.kStatorCurrentLimit = amps;
        controller.getConfigurator().apply(configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity).CurrentLimits, kCANTimeoutSeconds);
    }

    public void setPercentOutput(final double percent) {
        dutyCycleControl.Output = percent;
        controller.setControl(dutyCycleControl);
    }

    public void setVoltageOutput(final double voltage) {
        voltageControl.Output = voltage;
        controller.setControl(voltageControl);
    }

    public void setCurrentOutput(final double current, final double maxAbsoluteDutyCycle) {
        currentControl.Output = current;
        currentControl.MaxAbsDutyCycle = maxAbsoluteDutyCycle;
        controller.setControl(currentControl);
    }

    public void setPositionSetpoint(final int slot, final double setpoint) {
        setPositionSetpoint(slot, setpoint, 0.0);
    }

    public void setPositionSetpoint(final int slot, final double setpoint, final double feedforwardVolts) {
        positionControl.Slot = slot;
        positionControl.Position = toNativeSensorPosition(setpoint);
        positionControl.FeedForward = feedforwardVolts;
        controller.setControl(positionControl);
    }

    public void setMotionMagicPositionSetpoint(final boolean foc, final int slot, final double setpoint) {
        setMotionMagicPositionSetpoint(foc, slot, setpoint, 0.0);
    }

    public void setMotionMagicPositionSetpoint(final boolean foc, final int slot, final double setpoint, final double feedforwardVolts) {
        if(foc) {
            motionMagicFOCControl.Slot = slot;
            motionMagicFOCControl.Position = toNativeSensorPosition(setpoint);
            motionMagicFOCControl.FeedForward = feedforwardVolts;
            controller.setControl(motionMagicFOCControl);
        } else {
            motionMagicControl.Slot = slot;
            motionMagicControl.Position = toNativeSensorPosition(setpoint);
            motionMagicControl.FeedForward = feedforwardVolts;
            controller.setControl(motionMagicControl);
        }
    }

    public void setMotionMagicExpoPositionSetpoint(final boolean foc, final int slot, final double setpoint) {
        setMotionMagicExpoPositionSetpoint(foc, slot, setpoint, 0.0);
    }

    public void setMotionMagicExpoPositionSetpoint(final boolean foc, final int slot, final double setpoint, final double feedforwardVolts) {
        if(foc) {
            motionMagicExpoFOCControl.Slot = slot;
            motionMagicExpoFOCControl.Position = toNativeSensorPosition(setpoint);
            motionMagicExpoFOCControl.FeedForward = feedforwardVolts;
            controller.setControl(motionMagicExpoFOCControl);
        } else {
            motionMagicExpoControl.Slot = slot;
            motionMagicExpoControl.Position = toNativeSensorPosition(setpoint);
            motionMagicExpoControl.FeedForward = feedforwardVolts;
            controller.setControl(motionMagicExpoControl);
        }
    }

    public void setDynamicMotionMagicPositionSetpoint(final int slot, final double setpoint, final double velocity, final double acceleration, final double jerk) {
        setDynamicMotionMagicPositionSetpoint(slot, setpoint, velocity, acceleration, jerk, 0.0);
    }

    public void setDynamicMotionMagicPositionSetpoint(final int slot, final double setpoint, final double velocity, final double acceleration, final double jerk, final double feedforwardVolts) {
        dynamicMotionMagicControl.Slot = slot;
        dynamicMotionMagicControl.Position = toNativeSensorPosition(setpoint);
        dynamicMotionMagicControl.FeedForward = feedforwardVolts;
        dynamicMotionMagicControl.Velocity = toNativeSensorVelocity(velocity);
        dynamicMotionMagicControl.Acceleration = toNativeSensorVelocity(acceleration);
        dynamicMotionMagicControl.Jerk = toNativeSensorVelocity(jerk);
        controller.setControl(dynamicMotionMagicControl);
    }

    public void setVelocitySetpoint(final boolean foc, final int slot, final double setpointVelocity) {
        setVelocitySetpoint(foc, slot, setpointVelocity, 0.0, 0.0);
    }

    public void setVelocitySetpoint(final boolean foc, final int slot, final double setpointVelocity, final double feedforwardVolts) {
        setVelocitySetpoint(foc, slot, setpointVelocity, 0.0, feedforwardVolts);
    }

    public void setVelocitySetpoint(final boolean foc, final int slot, final double setpointVelocity, final double setpointAccel, final double feedforwardVolts) {
        if(foc) {
            velocityControlFOC.Slot = slot;
            velocityControlFOC.Velocity = toNativeSensorVelocity(setpointVelocity);
            velocityControlFOC.Acceleration = toNativeSensorVelocity(setpointAccel);
            velocityControlFOC.FeedForward = feedforwardVolts;
            controller.setControl(velocityControlFOC);
        } else {
            velocityControl.Slot = slot;
            velocityControl.Velocity = toNativeSensorVelocity(setpointVelocity);
            velocityControl.Acceleration = toNativeSensorVelocity(setpointAccel);
            velocityControl.FeedForward = feedforwardVolts;
            controller.setControl(velocityControl);
        }
    }

    public boolean getInverted() {
        return configuration.kInverted;
    }

    public void zeroSensorPosition() {
        setSensorPosition(0.0);
    }

    public void setSensorPosition(final double pos) {
        controller.setPosition(toNativeSensorPosition(pos));
    }

    public MechanismRatio getMechanismRatio() {
        return gearRatio;
    }

    public double toNativeSensorPosition(final double pos) {
        return toNativeSensorPosition(pos, gearRatio);
    }

    public static double toNativeSensorPosition(final double pos, final MechanismRatio mr) {
        // Native position is rotations. There is 1 rotation per revolution (lol).
        return mr.mechanismPositionToSensorRadians(pos) / (2.0 * Math.PI);
    }

    public double fromNativeSensorPosition(final double pos) {
        return (pos / toNativeSensorPosition(1.0, gearRatio));
    }

    public double toNativeSensorVelocity(final double vel) {
        return toNativeSensorVelocity(vel, gearRatio);
    }

    public static double toNativeSensorVelocity(final double vel, final MechanismRatio mr) {
        return toNativeSensorPosition(vel, mr);
    }

    public double fromNativeSensorVelocity(final double vel) {
        return vel / toNativeSensorVelocity(1.0);
    }

    public double toNativeSensorAcceleration(final double accel) {
        return toNativeSensorAcceleration(accel, gearRatio);
    }

    public static double toNativeSensorAcceleration(final double accel, final MechanismRatio mr) {
        return toNativeSensorVelocity(accel, mr);
    }

    public double fromNativeSensorAcceleration(final double accel) {
        return accel / toNativeSensorAcceleration(1.0);
    }

    public TalonFXSimState getSimulatedState() {
        return simulationState;
    }

    public void setSimulatedSensorPositionAndVelocity(final double position, final double velocity, final double dt, final MechanismRatio mr) {
        final double rotations = toNativeSensorPosition(position, mr);
        final double rotationsPerSecond = toNativeSensorVelocity(velocity, mr);
        final double sign = getInverted() ? -1.0 : 1.0;

        simulationState.setRotorVelocity(sign * rotationsPerSecond);
        simulationState.setRawRotorPosition(sign * rotations);
    }

    public void setSimulatedSensorVelocity(final double velocity, final double dt, final MechanismRatio mr) {
        final double rotationsPerSecond = toNativeSensorVelocity(velocity, mr);
        final double sign = getInverted() ? -1.0 : 1.0;

        simulationState.setRotorVelocity(sign * rotationsPerSecond);
        simulationState.addRotorPosition(sign * rotationsPerSecond * dt);
    }

    public void close() {
        controller.close();
    }

    public int getDeviceID() {
        return controller.getDeviceID();
    }
}