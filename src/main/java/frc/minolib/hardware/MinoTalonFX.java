package frc.minolib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

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
import frc.minolib.phoenix.PhoenixMotor;
import frc.minolib.phoenix.PhoenixUtility;
import frc.minolib.io.MotorInputsAutoLogged;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class MinoTalonFX implements AutoCloseable, PhoenixMotor {
    private static final double kCANTimeoutS = 0.1; // s
    private final String name;
    private final String loggingName;
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
    private final MotionMagicTorqueCurrentFOC motionMagicFOCControl = new MotionMagicTorqueCurrentFOC(0);
    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoFOCControl = new MotionMagicExpoTorqueCurrentFOC(0);
    private final DynamicMotionMagicVoltage dynamicMotionMagicControl = new DynamicMotionMagicVoltage(0, 0, 0, 0);

    private final MinoStatusSignal<Integer> faultFieldSignal;
    private final MinoStatusSignal<Integer> stickyFaultFieldSignal;
    private final MinoStatusSignal<Double> percentOutputSignal;
    private final MinoStatusSignal<Current> supplyCurrentSignal;
    private final MinoStatusSignal<Current> statorCurrentSignal;
    private final MinoStatusSignal<Current> torqueCurrentSignal;
    private final MinoStatusSignal<Angle> rotorPositionSignal;
    private final MinoStatusSignal<AngularVelocity> rotorVelocitySignal;
    private final MinoStatusSignal<AngularAcceleration> rotorAccelerationSignal;
    private final MinoStatusSignal<Angle> sensorPositionSignal;
    private final MinoStatusSignal<AngularVelocity> sensorVelocitySignal;
    private final MinoStatusSignal<AngularAcceleration> sensorAccelerationSignal;
    private final MinoStatusSignal<Double> closedLoopReferenceSignal;
    private final MinoStatusSignal<Double> closedLoopReferenceSlopeSignal;
    private final MinoStatusSignal<Temperature> temperatureSignal;
    private final BaseStatusSignal[] allSignals;

    private MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    private Alert disconnectedAlerts;
    private Alert overTempuratureAlert;
    private Alert overCurrentAlert;
    private Alert stallingAlert;

    public static class MinoTalonFXConfiguration {
        private NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        private boolean INVERTED = false;

        private double SUPPLY_CURRENT_LIMIT = 40.0; // A
        private double STATOR_CURRENT_LIMIT = 40.0; // A

        private boolean FWD_SOFT_LIMIT_ENABLED = false;
        private double FWD_SOFT_LIMIT = 0.0; // In MechanismRatio units
        private boolean REV_SOFT_LIMIT_ENABLED = false;
        private double REV_SOFT_LIMIT = 0.0; // In MechanismRatio units

        private PIDConfiguration slot0Configuration = new PIDConfiguration();
        private PIDConfiguration slot1Configuration = new PIDConfiguration();
        private PIDConfiguration slot2Configuration = new PIDConfiguration();

        private double motionMagicCruiseVelocity = 0.0; // In MechanismRatio units
        private double motionMagicAcceleration = 0.0; // In MechanismRatio units
        private double motionMagicJerk = 0.0; // In MechanismRatio units
        private double motionMagicExpokA = 0.0;
        private double motionMagicExpokV = 0.0;

        private CANDeviceID feedbackDeviceID = new CANDeviceID(0);
        private FeedbackSensorSourceValue feedbackSource = FeedbackSensorSourceValue.RemoteCANcoder;
        private boolean continuousWrappedEnabled = false;

        public MinoTalonFXConfiguration setBrakeMode() {
            NEUTRAL_MODE = NeutralModeValue.Brake;
            return this;
        }

        public MinoTalonFXConfiguration setInverted(final boolean inverted) {
            INVERTED = inverted;
            return this;
        }

        public MinoTalonFXConfiguration setStatorCurrentLimit(final double amperes) {
            STATOR_CURRENT_LIMIT = amperes;
            return this;
        }

        public MinoTalonFXConfiguration setSupplyCurrentLimit(final double amperes) {
            SUPPLY_CURRENT_LIMIT = amperes;
            return this;
        }

        public MinoTalonFXConfiguration setForwardSoftLimit(final double position) {
            FWD_SOFT_LIMIT_ENABLED = true;
            FWD_SOFT_LIMIT = position;
            return this;
        }

        public MinoTalonFXConfiguration setReverseSoftLimit(final double position) {
            REV_SOFT_LIMIT_ENABLED = true;
            REV_SOFT_LIMIT = position;
            return this;
        }

        public MinoTalonFXConfiguration setPIDConfig(final int slot, final PIDConfiguration configuration) {
            switch (slot) {
                case 0:
                    slot0Configuration = configuration;
                    break;
                case 1:
                    slot1Configuration = configuration;
                    break;
                case 2:
                    slot2Configuration = configuration;
                    break;
                default:
                    throw new RuntimeException("Invalid PID slot " + slot);
            }

            return this;
        }

        public MinoTalonFXConfiguration setMotionMagicConfig(final double cruiseVelocity, final double acceleration, final double jerk) {
            motionMagicCruiseVelocity = cruiseVelocity;
            motionMagicAcceleration = acceleration;
            motionMagicJerk = jerk;
            return this;
        }

        public MinoTalonFXConfiguration setMotionMagicExpoConfig(final double kA, final double kV) {
            motionMagicExpokA = kA;
            motionMagicExpokV = kV;
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
            final TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NEUTRAL_MODE;
            config.MotorOutput.Inverted = INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.DutyCycleNeutralDeadband = 0.0;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1; // s

            config.TorqueCurrent.PeakForwardTorqueCurrent = STATOR_CURRENT_LIMIT;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -STATOR_CURRENT_LIMIT;
            config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = FWD_SOFT_LIMIT_ENABLED;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = toNativeSensorPosition.apply(FWD_SOFT_LIMIT);
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = REV_SOFT_LIMIT_ENABLED;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = toNativeSensorPosition.apply(REV_SOFT_LIMIT);

            config.Voltage.SupplyVoltageTimeConstant = 0.0;
            config.Voltage.PeakForwardVoltage = 16.0;
            config.Voltage.PeakReverseVoltage = -16.0;

            config.Slot0 = slot0Configuration.fillCTRE(new Slot0Configs());
            config.Slot1 = slot1Configuration.fillCTRE(new Slot1Configs());
            config.Slot2 = slot2Configuration.fillCTRE(new Slot2Configs());

            config.MotionMagic.MotionMagicCruiseVelocity = toNativeSensorVelocity.apply(motionMagicCruiseVelocity);
            config.MotionMagic.MotionMagicAcceleration = toNativeSensorVelocity.apply(motionMagicAcceleration);
            config.MotionMagic.MotionMagicJerk = toNativeSensorVelocity.apply(motionMagicJerk);

            return config;
        }
    }

    public static MinoTalonFXConfiguration makeDefaultConfig() {
        return new MinoTalonFXConfiguration();
    }

    /** Follower constructor */
    public MinoTalonFX(final CANDeviceID canID, final MinoTalonFX leader, final MinoTalonFXConfiguration config) {
        this(canID, leader.getMechanismRatio(), config);
        controller.setControl(new StrictFollower(leader.getDeviceID()));
    }

    /** Constructor with full configuration */
    public MinoTalonFX(final CANDeviceID canID, final MechanismRatio gearRatio, final MinoTalonFXConfiguration configuration) {
        name = "TalonFX " + canID.toString();
        loggingName = "Inputs/" + name;
        controller = new TalonFX(canID.deviceNumber, canID.CANbusName);
        simulationState = controller.getSimState();
        this.gearRatio = gearRatio;
        this.configuration = configuration;

        disconnectedAlerts = new Alert("TalonFX " + canID.toString() + " is currently disconnected. Mechanism may not function as wanted", AlertType.kError);
        overCurrentAlert = new Alert("TalonFX " + canID.toString() + " is getting supplied too much power", AlertType.kWarning);
        overTempuratureAlert = new Alert("TalonFX " + canID.toString() + " is overheating, consider turning off the robot", AlertType.kWarning);
        stallingAlert = new Alert("TalonFX " + canID.toString() + " is currently stalling", AlertType.kInfo);

        faultFieldSignal = new MinoStatusSignal<>(controller.getFaultField());
        stickyFaultFieldSignal = new MinoStatusSignal<>(controller.getStickyFaultField());
        percentOutputSignal = new MinoStatusSignal<>(controller.getDutyCycle());
        supplyCurrentSignal = new MinoStatusSignal<>(controller.getSupplyCurrent());
        statorCurrentSignal = new MinoStatusSignal<>(controller.getStatorCurrent());
        torqueCurrentSignal = new MinoStatusSignal<>(controller.getTorqueCurrent());
        rotorPositionSignal = new MinoStatusSignal<>(controller.getPosition());
        rotorVelocitySignal = new MinoStatusSignal<>(controller.getVelocity());
        rotorAccelerationSignal = new MinoStatusSignal<>(controller.getAcceleration());
        sensorPositionSignal = new MinoStatusSignal<>(controller.getPosition(), this::fromNativeSensorPosition);
        sensorVelocitySignal = new MinoStatusSignal<>(controller.getVelocity(), this::fromNativeSensorVelocity);
        sensorAccelerationSignal = new MinoStatusSignal<>(controller.getAcceleration(), this::fromNativeSensorAcceleration);
        closedLoopReferenceSignal = new MinoStatusSignal<>(controller.getClosedLoopReference(), this::fromNativeSensorPosition);
        closedLoopReferenceSlopeSignal = new MinoStatusSignal<>(controller.getClosedLoopReferenceSlope(), this::fromNativeSensorVelocity);
        temperatureSignal = new MinoStatusSignal<>(controller.getDeviceTemp());

        allSignals = MinoStatusSignal.toBaseStatusSignals(
            faultFieldSignal,
            stickyFaultFieldSignal,
            percentOutputSignal,
            supplyCurrentSignal,
            statorCurrentSignal,
            torqueCurrentSignal,
            rotorPositionSignal,
            rotorVelocitySignal,
            rotorAccelerationSignal,
            sensorPositionSignal,
            sensorVelocitySignal,
            sensorAccelerationSignal,
            closedLoopReferenceSignal,
            closedLoopReferenceSlopeSignal,
            temperatureSignal
        );

        // Clear reset flag and sticky faults.
        controller.hasResetOccurred();
        controller.clearStickyFaults();

        Logger.recordOutput("Configuration/" + name, setConfiguration());
    }

    public boolean setConfiguration() {
        boolean allSuccess = true;

        // Set motor controller configuration.
        final TalonFXConfiguration config = configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity);
        allSuccess &= PhoenixUtility.retryUntilSuccess(() -> controller.getConfigurator().apply(config, kCANTimeoutS), () -> {
            TalonFXConfiguration readConfig = new TalonFXConfiguration();
            controller.getConfigurator().refresh(readConfig, kCANTimeoutS);
            return PhoenixUtility.TalonFXConfigsEqual(config, readConfig);
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

        final double kUpdateFrequency = 100.0; // Hz
        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> percentOutputSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> percentOutputSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": percentOutputSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> supplyCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> supplyCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": supplyCurrentSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> statorCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> statorCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": statorCurrentSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> torqueCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> torqueCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": torqueCurrentSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> rotorPositionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> rotorPositionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": rotorPositionSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> sensorPositionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> sensorPositionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": sensorPositionSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> sensorVelocitySignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> sensorVelocitySignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": sensorVelocitySignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> closedLoopReferenceSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> closedLoopReferenceSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": closedLoopReferenceSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> closedLoopReferenceSlopeSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> closedLoopReferenceSlopeSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": closedLoopReferenceSlopeSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> temperatureSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> temperatureSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": temperatureSignal.setUpdateFrequency()"
        );

        // Disable all signals that have not been explicitly defined.
        allSuccess &= PhoenixUtility.retryUntilSuccess(() -> controller.optimizeBusUtilization(0.0, kCANTimeoutS), name + ": optimizeBusUtilization");

        // Block until we get valid signals.
        allSuccess &= PhoenixUtility.retryUntilSuccess(() -> waitForInputs(kCANTimeoutS), name + ": waitForInputs()");

        // Check if unlicensed.
        allSuccess &= !controller.getStickyFault_UnlicensedFeatureInUse().getValue();
        return allSuccess;
    }

    public boolean checkFaultsAndReconfigureIfNecessary() {
        // TODO: Log other faults.
        if (controller.hasResetOccurred()) {
            DriverStation.reportError(name + ": reset occured", false);
            setConfiguration();
            return true;
        }

        return false;
    }

    public void close() {
        controller.close();
    }

    public int getDeviceID() {
        return controller.getDeviceID();
    }

    public StatusCode updateInputs() {
        disconnectedAlerts.set(!inputs.isMotorConnected);
        overTempuratureAlert.set(inputs.temperature > 95);
        overCurrentAlert.set(inputs.supplyCurrent > inputs.statorCurrent);
        stallingAlert.set(inputs.rotorVelocity < 10 && inputs.supplyCurrent > (inputs.statorCurrent / 2));
        
        return waitForInputs(0.0);
    }

    public StatusCode waitForInputs(final double timeoutSeconds) {
        inputs.isMotorConnected = BaseStatusSignal.isAllGood(allSignals);
        inputs.status = BaseStatusSignal.waitForAll(timeoutSeconds, allSignals);
        inputs.faultField = faultFieldSignal.getRawValue();
        inputs.stickyFaultField = stickyFaultFieldSignal.getRawValue();
        inputs.percentOutput = percentOutputSignal.getUnitConvertedValue();
        inputs.supplyCurrent = supplyCurrentSignal.getUnitConvertedValue();
        inputs.statorCurrent = statorCurrentSignal.getUnitConvertedValue();
        inputs.torqueCurrent = torqueCurrentSignal.getUnitConvertedValue();
        inputs.closedLoopReference = closedLoopReferenceSignal.getUnitConvertedValue();
        inputs.closedLoopReferenceSlope = closedLoopReferenceSlopeSignal.getUnitConvertedValue();
        inputs.rotorPosition = rotorPositionSignal.getUnitConvertedValue();
        inputs.rotorVelocity = rotorVelocitySignal.getUnitConvertedValue();
        inputs.rotorAcceleration = rotorAccelerationSignal.getUnitConvertedValue();
        inputs.sensorPosition = sensorPositionSignal.getUnitConvertedValue();
        inputs.sensorVelocity = sensorVelocitySignal.getUnitConvertedValue();
        inputs.sensorAcceleration = sensorAccelerationSignal.getUnitConvertedValue();
        inputs.latencyCompensatedSensorPosition = MinoStatusSignal.getLatencyCompensatedValue(sensorPositionSignal, sensorVelocitySignal);
        inputs.temperature = temperatureSignal.getUnitConvertedValue();

        Logger.processInputs(loggingName, inputs);
        return inputs.status;
    }

    public StatusCode setBrakeMode(final boolean on) {
        configuration.NEUTRAL_MODE = on ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        StatusCode ok = controller.getConfigurator().apply(configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity).MotorOutput);

        return ok;
    }

    public void setStatorCurrentLimit(final double amps) {
        configuration.STATOR_CURRENT_LIMIT = amps;

        // TODO: Consider a shorter non-blocking timeout
        controller.getConfigurator().apply(configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity).CurrentLimits, kCANTimeoutS);
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

    public StatusCode getStatus() {
        return inputs.status;
    }

    public BaseStatusSignal[] getOtherSignals() {
        return allSignals;
    }

    public double getPercentOutput() {
        return inputs.percentOutput;
    }

    public double getPhysicalPercentOutput() {
        return (getInverted() ? -1.0 : 1.0) * getPercentOutput();
    }

    public double getSupplyCurrent() {
        return inputs.supplyCurrent;
    }

    public double getStatorCurrent() {
        return inputs.statorCurrent;
    }

    public double getTorqueCurrent() {
        return inputs.torqueCurrent;
    }

    public double getClosedLoopReference() {
        return inputs.closedLoopReference;
    }

    public double getClosedLoopReferenceSlope() {
        return inputs.closedLoopReferenceSlope;
    }

    public double getMotorTemperature() {
        return inputs.temperature;
    }

    public boolean getInverted() {
        // This assumes that the config has been properly applied.
        return configuration.INVERTED;
    }

    public void zeroSensorPosition() {
        setSensorPosition(0.0);
    }

    public void setSensorPosition(final double pos) {
        // TODO: Handle zero offset internally.
        controller.setPosition(toNativeSensorPosition(pos));
    }

    public double getSensorPosition() {
        return inputs.sensorPosition;
    }

    public double getLatencyCompensatedSensorPosition() {
        return inputs.latencyCompensatedSensorPosition;
    }

    public double getSensorVelocity() {
        return inputs.sensorVelocity;
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
        // Native velocity is rotations per second.
        return toNativeSensorPosition(vel, mr);
    }

    public double fromNativeSensorVelocity(final double vel) {
        return vel / toNativeSensorVelocity(1.0);
    }

    public double toNativeSensorAcceleration(final double accel) {
        return toNativeSensorAcceleration(accel, gearRatio);
    }

    public static double toNativeSensorAcceleration(final double accel, final MechanismRatio mr) {
        // Native velocity is rotations per second.
        return toNativeSensorVelocity(accel, mr);
    }

    public double fromNativeSensorAcceleration(final double accel) {
        return accel / toNativeSensorAcceleration(1.0);
    }

    public TalonFXSimState getSimulatedState() {
        return simulationState;
    }

    public void setSimulatedSensorPositionAndVelocity(final double pos, final double vel, final double dt, final MechanismRatio mr) {
        // Convert position into rotations.
        final double rotations = toNativeSensorPosition(pos, mr);
        // Convert velocity into rotations per second.
        final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
        // Simulated hardware is never inverted, so flip signs accordingly.
        final double sign = getInverted() ? -1.0 : 1.0;
        simulationState.setRotorVelocity(sign * rotationsPerSecond);
        simulationState.setRawRotorPosition(sign * rotations);
    }

    public void setSimulatedSensorVelocity(final double vel, final double dt, final MechanismRatio mr) {
        // Convert velocity into rotations per second.
        final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
        // Simulated hardware is never inverted, so flip signs accordingly.
        final double sign = getInverted() ? -1.0 : 1.0;
        simulationState.setRotorVelocity(sign * rotationsPerSecond);
        simulationState.addRotorPosition(sign * rotationsPerSecond * dt);
    }
}