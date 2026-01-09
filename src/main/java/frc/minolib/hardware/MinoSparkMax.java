package frc.minolib.hardware;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.rev.ClosedLoopConfiguration;
import frc.minolib.rev.REVMotorController;
import frc.minolib.io.MotorInputsAutoLogged;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class MinoSparkMax implements  AutoCloseable, REVMotorController {
    private static final double kCANTimeoutS = 0.1; // s
    private final String name;
    private final String loggingName;
    private final SparkMax controller;
    private final SparkMaxSim simulationState;
    private final MechanismRatio gearRatio;
    private final MinoSparkMaxConfiguration configuration;

    private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    private Alert disconnectedAlert;
    private Alert overTempuratureAlert;
    private Alert overCurrentAlert;
    private Alert stallingAlert;

    public static class MinoSparkMaxConfiguration {
        private IdleMode IDLE_MODE = IdleMode.kCoast;
        private boolean INVERTED = false;
        private int PRIMARY_CURRENT_LIMIT = 40; // A
        private double SECONDARY_CURRENT_LIMIT = 40.0; // A
        private boolean FWD_SOFT_LIMIT_ENABLED = false;
        private double FWD_SOFT_LIMIT = 0.0; // In MechanismRatio units
        private boolean REV_SOFT_LIMIT_ENABLED = false;
        private double REV_SOFT_LIMIT = 0.0; // In MechanismRatio units
        private ClosedLoopConfiguration slot0Configuration = new ClosedLoopConfiguration();
        private ClosedLoopConfiguration slot1Configuration = new ClosedLoopConfiguration();
        private ClosedLoopConfiguration slot2Configuration = new ClosedLoopConfiguration();
        private double maxMotionCruiseVelocity = 0.0; // In MechanismRatio units
        private double maxMotionAcceleration = 0.0; // In MechanismRatio units
        private double bootPositionOffset = 0.0; // In MechanismRatio units
        private double rotorBootOffset = 0.0; // In rotor rotations [-1, 1]

        public MinoSparkMaxConfiguration setBrakeMode() {
            IDLE_MODE = IdleMode.kBrake;
            return this;
        }

        public MinoSparkMaxConfiguration setInverted(final boolean inverted) {
            INVERTED = inverted;
            return this;
        }

        public MinoSparkMaxConfiguration setPrimaryCurrentLimit(final int amperes) {
            PRIMARY_CURRENT_LIMIT = amperes;
            return this;
        }

        public MinoSparkMaxConfiguration setSecondaryCurrentLimit(final double amperes) {
            SECONDARY_CURRENT_LIMIT = amperes;
            return this;
        }

        public MinoSparkMaxConfiguration setForwardSoftLimit(final double position) {
            FWD_SOFT_LIMIT_ENABLED = true;
            FWD_SOFT_LIMIT = position;
            return this;
        }

        public MinoSparkMaxConfiguration setReverseSoftLimit(final double position) {
            REV_SOFT_LIMIT_ENABLED = true;
            REV_SOFT_LIMIT = position;
            return this;
        }

        public MinoSparkMaxConfiguration setPIDConfiguration(final ClosedLoopSlot slot, final ClosedLoopConfiguration configuration) {
            switch (slot) {
                case kSlot0:
                    slot0Configuration = configuration;
                    break;
                case kSlot1:
                    slot1Configuration = configuration;
                    break;
                case kSlot2:
                    slot2Configuration = configuration;
                    break;
                case kSlot3:
                    slot2Configuration = configuration;
                    break;
                default:
                    throw new RuntimeException("Invalid PID slot " + slot);
            }

            return this;
        }

        public MinoSparkMaxConfiguration setMaxMotionConfiguration(final double cruiseVelocity, final double acceleration) {
            maxMotionCruiseVelocity = cruiseVelocity;
            maxMotionAcceleration = acceleration;
            return this;
        }

        public MinoSparkMaxConfiguration setBootPositionOffset(final double position) {
            bootPositionOffset = position;
            return this;
        }

        public MinoSparkMaxConfiguration setRotorBootOffset(final double position) {
            rotorBootOffset = position;
            return this;
        }

        public SparkMaxConfig toSparkMaxConfiguration(final Function<Double, Double> toNativeSensorPosition, final Function<Double, Double> toNativeSensorVelocity) {
            final SparkMaxConfig configuration = new SparkMaxConfig();

            configuration.idleMode(IDLE_MODE);
            configuration.inverted(INVERTED);

            configuration.smartCurrentLimit(PRIMARY_CURRENT_LIMIT);
            configuration.secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);

            configuration.softLimit.forwardSoftLimitEnabled(FWD_SOFT_LIMIT_ENABLED);
            configuration.softLimit.forwardSoftLimit(toNativeSensorPosition.apply(FWD_SOFT_LIMIT));
            configuration.softLimit.reverseSoftLimitEnabled(REV_SOFT_LIMIT_ENABLED);
            configuration.softLimit.reverseSoftLimit(toNativeSensorPosition.apply(REV_SOFT_LIMIT));

            configuration.closedLoop.maxMotion.maxVelocity(toNativeSensorVelocity.apply(maxMotionCruiseVelocity));
            configuration.closedLoop.maxMotion.maxAcceleration(toNativeSensorVelocity.apply(maxMotionAcceleration));

            configuration.closedLoop.maxMotion.apply(slot0Configuration.fillREV(new MAXMotionConfig()));
            configuration.closedLoop.apply(slot1Configuration.fillREV(new ClosedLoopConfig()));

            //config.Feedback.FeedbackRotorOffset = rotorBootOffset;

            return configuration;
        }
    }

    public static MinoSparkMaxConfiguration makeDefaultConfig() {
        return new MinoSparkMaxConfiguration();
    }

    /** Follower constructor */
    public MinoSparkMax(final CANDeviceID canID, final MinoSparkMax leader, final MinoSparkMaxConfiguration configuration) {
        this(canID, leader.getMechanismRatio(), configuration);
        //controller.setControl(new StrictFollower(leader.getDeviceID()));
    }

    /** Constructor with full configuration */
    public MinoSparkMax(final CANDeviceID canID, final MechanismRatio gearRatio, final MinoSparkMaxConfiguration configuration) {
        name = "SparkMax " + canID.toString();
        loggingName = "Inputs/" + name;
        controller = new SparkMax(canID.deviceNumber, MotorType.kBrushless);
        simulationState = new SparkMaxSim(controller, DCMotor.getNEO(1));
        this.gearRatio = gearRatio;
        this.configuration = configuration;

        disconnectedAlert = new Alert("SparkMax [" + canID.toString() + "] is current disconnected. Mechanism may not funciton as wanted", AlertType.kError);
        overCurrentAlert = new Alert("SparkMax " + canID.toString() + " is getting supplied too much power", AlertType.kWarning);
        overTempuratureAlert = new Alert("SparkMax " + canID.toString() + " is overheating, consider turning off the robot", AlertType.kWarning);
        stallingAlert = new Alert("SparkMax " + canID.toString() + " is currently stalling", AlertType.kInfo);

        // Clear reset flag and sticky faults.
        controller.hasActiveFault();
        controller.clearFaults();

        Logger.recordOutput("Configuration/" + name, setConfiguration());
    }

    public boolean setConfiguration() {
        boolean allSuccess = true;

        // Set motor controller configuration.
        final SparkMaxConfig configuration = this.configuration.toSparkMaxConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity);
        allSuccess &= controller.configureAsync(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters).equals(REVLibError.kOk);

        allSuccess &= controller.getFaults().can;

        // Block until we get valid signals.
        allSuccess &= controller.getFaults().sensor;
        allSuccess &= controller.getFaults().can;

        return allSuccess;
    }

    public boolean checkFaultsAndReconfigureIfNecessary() {
        // TODO: Log other faults.
        if (controller.hasActiveFault()) {
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
        return controller.getDeviceId();
    }

    @Override
    public void updateInputs() {
        inputs.isMotorConnected = !controller.getFaults().can && !controller.getFaults().sensor && !controller.getFaults().temperature;
        inputs.faultField = controller.getFaults().rawBits;
        inputs.stickyFaultField = controller.getStickyFaults().rawBits;
        inputs.percentOutput = controller.get();
        inputs.supplyCurrent = controller.getOutputCurrent();
        //inputs.closedLoopReference = controller.getClosedLoopController();
        inputs.rotorPosition = controller.getEncoder().getPosition();
        inputs.sensorPosition = toNativeSensorPosition(controller.getEncoder().getPosition());
        inputs.sensorVelocity = toNativeSensorVelocity(controller.getEncoder().getVelocity());

        disconnectedAlert.set(controller.hasStickyFault());
        overTempuratureAlert.set(controller.getMotorTemperature() > 90);
        overCurrentAlert.set(controller.getStickyWarnings().overcurrent);
        stallingAlert.set(toNativeSensorVelocity(controller.getEncoder().getVelocity()) < 10 && controller.getOutputCurrent() > 20); // TODO: Needs some work

        Logger.processInputs(loggingName, inputs);
    }

    public void setBrakeMode(final boolean on) {
        configuration.IDLE_MODE = on ? IdleMode.kBrake : IdleMode.kCoast;
        controller.configure(configuration.toSparkMaxConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPrimaryCurrentLimit(final int amps) {
        configuration.PRIMARY_CURRENT_LIMIT = amps;
        controller.configure(configuration.toSparkMaxConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPercentOutput(final double percent) {
        controller.set(percent);
    }

    public void setVoltageOutput(final double voltage) {
        controller.setVoltage(voltage);
    }


    public void setPositionSetpoint(final ClosedLoopSlot slot, final double setpoint) {
        setPositionSetpoint(slot, setpoint, 0.0);
    }

    public void setPositionSetpoint(final ClosedLoopSlot slot, final double setpoint, final double feedforwardVolts) {
        controller.getClosedLoopController().setReference(toNativeSensorPosition(setpoint), ControlType.kPosition, slot, feedforwardVolts);
    }

    public void setVelocitySetpoint(final ClosedLoopSlot slot, final double setpointVelocity) {
        setVelocitySetpoint(slot, setpointVelocity, 0.0);
    }

    public void setVelocitySetpoint(final ClosedLoopSlot slot, final double setpointVelocity, final double feedforwardVolts) {
        controller.getClosedLoopController().setReference(toNativeSensorVelocity(setpointVelocity), ControlType.kVelocity, slot, feedforwardVolts);
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

    public double getClosedLoopReference() {
        return inputs.closedLoopReference;
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
        controller.getEncoder().setPosition(toNativeSensorPosition(pos));
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
        return toNativeSensorPosition(pos, gearRatio, configuration.bootPositionOffset);
    }

    public static double toNativeSensorPosition(final double pos, final MechanismRatio mr, final double bootPositionOffset) {
        // Native position is rotations. There is 1 rotation per revolution (lol).
        return mr.mechanismPositionToSensorRadians(pos - bootPositionOffset) / (2.0 * Math.PI);
    }

    public double fromNativeSensorPosition(final double pos) {
        return (pos / toNativeSensorPosition(1.0, gearRatio, 0.0)) + configuration.bootPositionOffset;
    }

    public double toNativeSensorVelocity(final double vel) {
        return toNativeSensorVelocity(vel, gearRatio);
    }

    public static double toNativeSensorVelocity(final double vel, final MechanismRatio mr) {
        // Native velocity is rotations per second.
        return toNativeSensorPosition(vel, mr, 0.0);
    }

    public double fromNativeSensorVelocity(final double vel) {
        return vel / toNativeSensorVelocity(1.0);
    }

    public void setSimulatedSensorPositionAndVelocity(final double pos, final double vel, final double dt, final MechanismRatio mr) {
        // Convert position into rotations.
        final double rotations = toNativeSensorPosition(pos, mr, 0.0);
        // Convert velocity into rotations per second.
        final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
        // Simulated hardware is never inverted, so flip signs accordingly.
        final double sign = getInverted() ? -1.0 : 1.0;
        simulationState.setVelocity(sign * rotationsPerSecond);
        simulationState.setPosition(sign * rotations);
    }

    public void setSimulatedSensorVelocity(final double vel, final double dt, final MechanismRatio mr) {
        // Convert velocity into rotations per second.
        final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
        // Simulated hardware is never inverted, so flip signs accordingly.
        final double sign = getInverted() ? -1.0 : 1.0;
        simulationState.setVelocity(sign * rotationsPerSecond);
        simulationState.setPosition(sign * rotationsPerSecond * dt);
    }
}