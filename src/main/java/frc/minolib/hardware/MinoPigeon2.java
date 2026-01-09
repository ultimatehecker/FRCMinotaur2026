package frc.minolib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.fasterxml.jackson.databind.type.ClassKey;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.minolib.io.GyroInputs;
import frc.minolib.io.GyroInputsAutoLogged;
import frc.minolib.phoenix.MinoStatusSignal;
import frc.minolib.phoenix.PhoenixGyro;
import frc.minolib.phoenix.PhoenixUtility;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

/**
 * The MinoPigeon2 class is an implementation of the PhoenixIO and MinoIMU interfaces, providing an
 * interface to the Pigeon2 IMU sensor. It includes methods for configuring the sensor, updating
 * input values, and retrieving various sensor readings such as roll, pitch, yaw, and their
 * respective rates.
 *
 * <p>Features:
 *
 * <ul>
 *   <li>Configuration of the Pigeon2 sensor with custom settings.
 *   <li>Automatic logging of sensor inputs.
 *   <li>Methods to retrieve roll, pitch, yaw, and their rates.
 *   <li>Support for continuous yaw with offset adjustment.
 *   <li>Simulation support for continuous yaw.
 * </ul>
 *
 * <p>Usage:
 *
 * <pre>{@code
 * // Create a MinoPigeon2 instance with default configuration
 * MinoPigeon2 pigeon = new MinoPigeon2(canID);
 *inputsnfiguration();
 * config.setGyroTrimZ(Math.toRadians(5.0));
 *
 * // Create a MinoPigeon2 instance with custom configuration
 * MinoPigeon2 pigeon = new MinoPigeon2(canID, config);
 * }</pre>
 *
 * <p>Methods:
 *
 * <ul>
 *   <li>{@link #setConfiguration()} - Applies the configuration to the Pigeon2 sensor.
 *   <li>{@link #updateInputs()} - Updates the input values from the sensor.
 *   <li>{@link #waitForInputs(double)} - Waits for input values to be updated with a timeout.
 *   <li>{@link #zeroContinuousYaw()} - Resets the continuous yaw to zero.
 *   <li>{@link #setContinuousYaw(double)} - Sets the continuous yaw to a specified value.
 *   <li>{@link #getRoll()} - Retrieves the roll value.
 *   <li>{@link #getPitch()} - Retrieves the pitch value.
 *   <li>{@link #getContinuousYaw()} - Retrieves the continuous yaw value.
 *   <li>{@link #getLatencyCompensatedContinuousYaw()} - Retrieves the latency-compensated
 *       continuous yaw value.
 *   <li>{@link #getRollRate()} - Retrieves the roll rate value.
 *   <li>{@link #getPitchRate()} - Retrieves the pitch rate value.
 *   <li>{@link #getYawRate()} - Retrieves the yaw rate value.
 *   <li>{@link #setSimContinuousYaw(double)} - Sets the simulated continuous yaw value.
 * </ul>
 *
 * <p>Inner Classes:
 *
 * <ul>
 *   <li>{@link MinoPigeon2.Pigeon2Inputs} - Holds the input values from the Pigeon2 sensor.
 *   <li>{@link MinoPigeon2.MinoPigeon2Configuration} - Configuration class for the Pigeon2 sensor.
 * </ul>
 *
 * @see PhoenixIO
 * @see MinoGyro
 */

public class MinoPigeon2 implements AutoCloseable, PhoenixGyro {
  private static final double kCANTimeoutS = 0.1; // s
  private final String name;
  private final String loggingName;
  private final Pigeon2 pigeon;
  private final Pigeon2SimState simulationState;
  private final MinoPigeon2Configuration configuration;

  private final MinoStatusSignal<Integer> faultFieldSignal;
  private final MinoStatusSignal<Integer> stickyFaultFieldSignal;
  private final MinoStatusSignal<Angle> rollSignal;
  private final MinoStatusSignal<Angle> pitchSignal;
  private final MinoStatusSignal<Angle> yawSignal;
  private final MinoStatusSignal<AngularVelocity> rollRateSignal;
  private final MinoStatusSignal<AngularVelocity> pitchRateSignal;
  private final MinoStatusSignal<AngularVelocity> yawRateSignal;
  private final BaseStatusSignal[] allSignals;

  private double continuousYawOffset = 0.0;

  private final GyroInputsAutoLogged inputs = new GyroInputsAutoLogged();

  private Alert disconnectedAlert;
  private Alert overTempuratureAlert;
  private Alert supplyVoltageAlert;

  /**
   * Configuration class for MinoPigeon2 that handles mount pose and gyro trim settings. All angle
   * measurements are in radians internally but converted to degrees when creating the
   * Pigeon2Configuration.
   *
   * <p>Mount pose represents the physical orientation of the Pigeon2 sensor relative to the robot,
   * while gyro trim values are used for calibration adjustments.
   *
   * @see com.ctre.phoenix.sensors.Pigeon2Configuration
   */
  public static class MinoPigeon2Configuration {
    private double mountPoseRoll = 0.0; // rads
    private double mountPosePitch = 0.0; // rads
    private double mountPoseYaw = 0.0; // rads
    private double gyroTrimX = 0.0; // rads
    private double gyroTrimY = 0.0; // rads
    private double gyroTrimZ = 0.0; // rads

    public MinoPigeon2Configuration setGyroTrimZ(final double rads) {
      gyroTrimZ = rads;
      return this;
    }

    public Pigeon2Configuration toPigeon2Configuration() {
      Pigeon2Configuration config = new Pigeon2Configuration();

      config.MountPose.MountPoseRoll = Math.toDegrees(mountPoseYaw);
      config.MountPose.MountPosePitch = Math.toDegrees(mountPosePitch);
      config.MountPose.MountPoseYaw = Math.toDegrees(mountPoseRoll);

      config.GyroTrim.GyroScalarX = Math.toDegrees(gyroTrimX);
      config.GyroTrim.GyroScalarY = Math.toDegrees(gyroTrimY);
      config.GyroTrim.GyroScalarZ = Math.toDegrees(gyroTrimZ);

      return config;
    }
  }

  public static MinoPigeon2Configuration makeDefaultConfig() {
    return new MinoPigeon2Configuration();
  }

  /** Default constructor */
  public MinoPigeon2(final CANDeviceID canID) {
    this(canID, makeDefaultConfig());
  }

  /**
   * A wrapper class for CTRE's Pigeon2 IMU sensor with additional functionality.
   *
   * @param canID The CAN ID configuration for the Pigeon2 device
   * @param config The configuration settings for the Pigeon2
   *     <p>The constructor: - Initializes the Pigeon2 device with the given CAN ID - Sets up status
   *     signals for faults, orientation (roll, pitch, yaw), and angular velocities - Converts
   *     angular measurements from degrees to radians - Clears any previous reset flags and sticky
   *     faults - Logs the device configuration
   */
  public MinoPigeon2(final CANDeviceID canID, final MinoPigeon2Configuration config) {
    name = "Pigeon2 " + canID.toString();
    loggingName = "Inputs/" + name;
    pigeon = new Pigeon2(canID.deviceNumber, canID.CANbusName);
    simulationState = pigeon.getSimState();
    configuration = config;

    disconnectedAlert = new Alert("Pigeon [" + canID.toString() + "] is currently disconnected. Mechanism may not function as wanted", AlertType.kError);
    overTempuratureAlert = new Alert("Pigeon [" + canID.toString() + "] is overheating, consider turning off the robot", AlertType.kWarning);
    supplyVoltageAlert = new Alert("CANcoder [" + canID.toString() + "] is underpowered. Device might be permentally damaged", AlertType.kWarning);

    faultFieldSignal = new MinoStatusSignal<>(pigeon.getFaultField());
    stickyFaultFieldSignal = new MinoStatusSignal<>(pigeon.getStickyFaultField());
    rollSignal = new MinoStatusSignal<>(pigeon.getRoll(), Math::toRadians);
    pitchSignal = new MinoStatusSignal<>(pigeon.getPitch(), Math::toRadians);
    yawSignal = new MinoStatusSignal<>(pigeon.getYaw(), (Double value) -> { return Math.toRadians(value) - continuousYawOffset; });
    rollRateSignal = new MinoStatusSignal<>(pigeon.getAngularVelocityXDevice(), Math::toRadians);
    pitchRateSignal = new MinoStatusSignal<>(pigeon.getAngularVelocityYDevice(), Math::toRadians);
    yawRateSignal = new MinoStatusSignal<>(pigeon.getAngularVelocityZDevice(), Math::toRadians);
    allSignals = MinoStatusSignal.toBaseStatusSignals(
      faultFieldSignal,
      stickyFaultFieldSignal,
      rollSignal,
      pitchSignal,
      yawSignal,
      rollRateSignal,
      pitchRateSignal,
      yawRateSignal
    );

    // Clear reset flag and sticky faults.
    pigeon.hasResetOccurred();
    pigeon.clearStickyFaults();

    Logger.recordOutput("Configuration/" + name, setConfiguration());
  }

  /**
   * Configures the Pigeon2 IMU device with specified settings and update frequencies.
   *
   * <p>This method: 1. Applies the Pigeon2 configuration 2. Sets update frequencies for fault
   * signals (4Hz) 3. Sets update frequencies for motion signals (100Hz) 4. Waits for valid input
   * signals 5. Checks for unlicensed feature usage
   *
   * <p>Note: Bus utilization optimization is currently disabled due to potential stale signal
   * issues.
   *
   * @return boolean indicating whether all configuration steps were successful (true) or if any
   *     failed (false)
   */
  public boolean setConfiguration() {
    boolean allSuccess = true;

    // Set configuration.
    Pigeon2Configuration config = configuration.toPigeon2Configuration();
    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> pigeon.getConfigurator().apply(config, kCANTimeoutS), () -> {
      Pigeon2Configuration readConfig = new Pigeon2Configuration();
      pigeon.getConfigurator().refresh(readConfig, kCANTimeoutS);
      return PhoenixUtility.Pigeon2ConfigsEqual(config, readConfig);
    }, name + ": applyConfiguration");

    // Set update frequencies.
    final double kFaultUpdateFrequency = 4.0; // Hz
    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> faultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
      () -> faultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
      name + ": faultFieldSignal.setUpdateFrequency()"
    );

    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> stickyFaultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
      () -> stickyFaultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
      name + ": stickyFaultFieldSignal.setUpdateFrequency()"
    );

    final double kUpdateFrequency = 100.0; // Hz
    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> rollSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
      () -> rollSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
      name + ": rollSignal.setUpdateFrequency()"
    );

    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> pitchSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
      () -> pitchSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
      name + ": pitchSignal.setUpdateFrequency()"
    );

    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> yawSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
      () -> yawSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
      name + ": yawSignal.setUpdateFrequency()"
    );

    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> rollRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
      () -> rollRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
      name + ": rollRateSignal.setUpdateFrequency()"
    );

    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> pitchRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
      () -> pitchRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
      name + ": pitchRateSignal.setUpdateFrequency()"
    );

    allSuccess &= PhoenixUtility.retryUntilSuccess(() -> yawRateSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
      () -> yawRateSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
      name + ": yawRateSignal.setUpdateFrequency()"
    );

    // Disable all signals that have not been explicitly defined.
    // TODO: Figure out why this sometimes causes stale signals.
    // allSuccess &=
    //     PhoenixUtil.retryUntilSuccess(
    //         () -> pigeon.optimizeBusUtilization(kCANTimeoutS),
    //         name + ": optimizeBusUtilization");

    // Block until we get valid signals.
    allSuccess &= BaseStatusSignal.waitForAll(kCANTimeoutS, allSignals).isOK();

    // Check if unlicensed.
    allSuccess &= !pigeon.getStickyFault_UnlicensedFeatureInUse().getValue();

    return allSuccess;
  }

  public void close() {
    pigeon.close();
  }

  public StatusCode updateInputs() {
    disconnectedAlert.set(inputs.isGyroConnected);
    overTempuratureAlert.set(pigeon.getTemperature().getValue().in(Celsius) > 95);
    supplyVoltageAlert.set(pigeon.getSupplyVoltage().getValue().in(Volts) < 11.5);
    
    return waitForInputs(0.0);
  }

  /**
   * Waits for all status signals from the Pigeon2 and updates input values. This method collects
   * data for roll, pitch, yaw, and their respective rates, as well as fault information from the
   * sensor.
   *
   * @param timeoutSec The maximum time to wait for signals in seconds
   * @return StatusCode indicating the result of waiting for signals: - OK if successful - TIMEOUT
   *     if the wait operation times out - ERROR if there was a communication error
   */
  public StatusCode waitForInputs(final double timeoutSeconds) {
    inputs.isGyroConnected = BaseStatusSignal.isAllGood(allSignals);
    inputs.status = BaseStatusSignal.waitForAll(0.0, allSignals);
    inputs.faultField = faultFieldSignal.getRawValue();
    inputs.stickyFaultField = stickyFaultFieldSignal.getRawValue();
    inputs.roll = rollSignal.getUnitConvertedValue();
    inputs.pitch = pitchSignal.getUnitConvertedValue();
    inputs.yaw = yawSignal.getUnitConvertedValue();
    inputs.latencyCompensatedYaw = MinoStatusSignal.getLatencyCompensatedValue(yawSignal, yawRateSignal);
    inputs.rollRate = rollRateSignal.getUnitConvertedValue();
    inputs.pitchRate = pitchRateSignal.getUnitConvertedValue();
    inputs.yawRate = yawRateSignal.getUnitConvertedValue();
    inputs.rotation3D = pigeon.getRotation3d();

    Logger.processInputs(loggingName, inputs);
    return inputs.status;
  }

  /**
   * Resets the continuous yaw measurement to zero. This is equivalent to calling
   * setContinuousYaw(0.0).
   */
  public void zeroContinuousYaw() {
    setContinuousYaw(0.0);
  }

  /**
   * Sets the continuous yaw (heading) of the Pigeon2 to a specified angle. Adjusts the continuous
   * yaw offset to maintain continuous rotation tracking.
   *
   * @param rad The desired yaw angle in radians to set the Pigeon2 to
   */
  public void setContinuousYaw(final double rad) {
    continuousYawOffset += getContinuousYaw() - rad;
    updateInputs();
  }

  public StatusCode getStatus() {
    return inputs.status;
  }

  public BaseStatusSignal[] getStatusSignals() {
    return allSignals;
  }

  public double getRoll() {
    return inputs.roll;
  }

  public double getPitch() {
    return inputs.pitch;
  }

  public double getContinuousYaw() {
    return inputs.yaw;
  }

  public double getLatencyCompensatedContinuousYaw() {
    return inputs.latencyCompensatedYaw;
  }

  public double getRollRate() {
    return inputs.rollRate;
  }

  public double getPitchRate() {
    return inputs.pitchRate;
  }

  public double getYawRate() {
    return inputs.yawRate;
  }

  public Rotation3d getRotation3d() {
    return inputs.rotation3D;
  }

  public Pigeon2SimState getSimulatedState() {
    return simulationState;
  }

  public void setSimContinuousYaw(final double rad) {
    simulationState.setRawYaw(Math.toDegrees(rad));
  }

  public void setSimYawRate(final double radsPerSec) {
    simulationState.setAngularVelocityZ(Math.toDegrees(radsPerSec));
  }
}