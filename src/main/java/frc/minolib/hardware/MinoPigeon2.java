package frc.minolib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

import frc.minolib.phoenix.MinoStatusSignal;
import frc.minolib.phoenix.PhoenixUtility;

public class MinoPigeon2 implements AutoCloseable {
  private static final double kCANTimeoutSeconds = 0.1;

  private final String name;
  private final Pigeon2 pigeon;
  private final Pigeon2SimState simulationState;

  private final MinoStatusSignal<Integer> faultFieldSignal;
  private final MinoStatusSignal<Integer> stickyFaultFieldSignal;

  private final MinoStatusSignal<Angle> rollSignal;
  private final MinoStatusSignal<Angle> pitchSignal;
  private final MinoStatusSignal<Angle> yawSignal;

  private final MinoStatusSignal<AngularVelocity> rollVelocitySignal;
  private final MinoStatusSignal<AngularVelocity> pitchVelocitySignal;
  private final MinoStatusSignal<AngularVelocity> yawVelocitySignal;

  private final MinoStatusSignal<LinearAcceleration> accelerationX;
  private final MinoStatusSignal<LinearAcceleration> accelerationY;
  private final MinoStatusSignal<LinearAcceleration> accelerationZ;

  private final MinoStatusSignal<Voltage> supplyVoltage;
  private final MinoStatusSignal<Temperature> temperature;

  private final Alert unlicensedFeatureAlert;

  private Rotation2d lastYaw = Rotation2d.fromRadians(0.0);
  private Rotation2d continuousYaw = new Rotation2d(0.0);

  public static class MinoPigeon2Configuration {
    private double mountPoseRoll = 0.0;
    private double mountPosePitch = 0.0;
    private double mountPoseYaw = 0.0;
    private double gyroTrimX = 0.0;
    private double gyroTrimY = 0.0;
    private double gyroTrimZ = 0.0;

    public MinoPigeon2Configuration setGyroTrimZ(final double rads) {
      gyroTrimZ = rads;
      return this;
    }

    public Pigeon2Configuration toPigeon2Configuration() {
      Pigeon2Configuration configuration = new Pigeon2Configuration();

      configuration.MountPose.MountPoseRoll = Math.toDegrees(mountPoseYaw);
      configuration.MountPose.MountPosePitch = Math.toDegrees(mountPosePitch);
      configuration.MountPose.MountPoseYaw = Math.toDegrees(mountPoseRoll);

      configuration.GyroTrim.GyroScalarX = Math.toDegrees(gyroTrimX);
      configuration.GyroTrim.GyroScalarY = Math.toDegrees(gyroTrimY);
      configuration.GyroTrim.GyroScalarZ = Math.toDegrees(gyroTrimZ);

      return configuration;
    }
  }

  public static MinoPigeon2Configuration makeDefaultConfig() {
    return new MinoPigeon2Configuration();
  }

  public MinoPigeon2(final CANDeviceID canID) {
    this(canID, makeDefaultConfig());
  }

  public MinoPigeon2(final CANDeviceID canID, final MinoPigeon2Configuration configuration) {
    name = "Pigeon2 " + canID.toString();
    pigeon = new Pigeon2(canID.deviceNumber, canID.CANbusName);
    simulationState = pigeon.getSimState();

    faultFieldSignal = new MinoStatusSignal<>(pigeon.getFaultField());
    stickyFaultFieldSignal = new MinoStatusSignal<>(pigeon.getStickyFaultField());

    rollSignal = new MinoStatusSignal<>(pigeon.getRoll(), Math::toRadians);
    pitchSignal = new MinoStatusSignal<>(pigeon.getPitch(), Math::toRadians);
    yawSignal = new MinoStatusSignal<>(pigeon.getYaw(), Math::toRadians);

    rollVelocitySignal = new MinoStatusSignal<>(pigeon.getAngularVelocityXDevice(), Math::toRadians);
    pitchVelocitySignal = new MinoStatusSignal<>(pigeon.getAngularVelocityYDevice(), Math::toRadians);
    yawVelocitySignal = new MinoStatusSignal<>(pigeon.getAngularVelocityZDevice(), Math::toRadians);

    accelerationX = new MinoStatusSignal<>(pigeon.getAccelerationX());
    accelerationY = new MinoStatusSignal<>(pigeon.getAccelerationY());
    accelerationZ = new MinoStatusSignal<>(pigeon.getAccelerationZ());

    supplyVoltage = new MinoStatusSignal<>(pigeon.getSupplyVoltage());
    temperature = new MinoStatusSignal<>(pigeon.getTemperature());

    unlicensedFeatureAlert = new Alert(name + " is using an unlicensed feature. Device behavior may be restricted.", AlertType.kError);

    pigeon.hasResetOccurred();
    pigeon.clearStickyFaults();

    applyConfiguration(configuration);
  }

  public boolean applyConfiguration(MinoPigeon2Configuration configuration) {
    boolean allSuccess = true;

    final Pigeon2Configuration config = configuration.toPigeon2Configuration();
      allSuccess &= PhoenixUtility.retryUntilSuccess(
        () -> pigeon.getConfigurator().apply(config, kCANTimeoutSeconds),
        () -> {
          Pigeon2Configuration readConfig = new Pigeon2Configuration();
          pigeon.getConfigurator().refresh(readConfig, kCANTimeoutSeconds);
          return PhoenixUtility.Pigeon2ConfigsEqual(config, readConfig);
        },
        name + ": applyConfiguration"
      );

      allSuccess &= PhoenixUtility.retryUntilSuccess(
        () -> pigeon.optimizeBusUtilization(0.0, kCANTimeoutSeconds),
        name + ": optimizeBusUtilization"
      );

    final StatusSignal<Boolean> unlicensedSignal = pigeon.getStickyFault_UnlicensedFeatureInUse();
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
    return rollSignal;
  }

  public MinoStatusSignal<Angle> getPitchSignal() {
    return pitchSignal;
  }

  public MinoStatusSignal<Angle> getYawSignal() {
    return yawSignal;
  }

  public MinoStatusSignal<AngularVelocity> getRollRateSignal() {
    return rollVelocitySignal;
  }

  public MinoStatusSignal<AngularVelocity> getPitchRateSignal() {
    return pitchVelocitySignal;
  }

  public MinoStatusSignal<AngularVelocity> getYawRateSignal() {
    return yawVelocitySignal;
  }

  public MinoStatusSignal<LinearAcceleration> getAccelerationX() {
    return accelerationX;
  }

  public MinoStatusSignal<LinearAcceleration> getAccelerationY() {
    return accelerationY;
  }

  public MinoStatusSignal<LinearAcceleration> getAccelerationZ() {
    return accelerationZ;
  }

  public MinoStatusSignal<Voltage> getSupplyVoltage() {
    return supplyVoltage;
  }

  public MinoStatusSignal<Temperature> getTemperature() {
    return temperature;
  }

  public BaseStatusSignal[] getAllSignals() {
    return MinoStatusSignal.toBaseStatusSignals(
      faultFieldSignal,
      stickyFaultFieldSignal,
      rollSignal,
      pitchSignal,
      yawSignal,
      rollVelocitySignal,
      pitchVelocitySignal,
      yawVelocitySignal,
      accelerationX,
      accelerationY,
      accelerationZ,
      supplyVoltage,
      temperature
    );
  }

  public void updateContinuousYaw() {
    final Rotation2d currentYaw = Rotation2d.fromRadians(yawSignal.getUnitConvertedValue());

    continuousYaw = continuousYaw.plus(currentYaw.minus(lastYaw));
    lastYaw = currentYaw;
  }

  public void zeroContinuousYaw() {
    setContinuousYaw(Rotation2d.fromRadians(0.0));
  }

  public void setContinuousYaw(final Rotation2d heading) {
    continuousYaw = heading.plus(continuousYaw.minus(
        Rotation2d.fromRadians(yawSignal.getUnitConvertedValue())
    ));

    lastYaw = Rotation2d.fromRadians(yawSignal.getUnitConvertedValue());
  }

  public Rotation2d getContinuousYaw() {
    return continuousYaw;
  }

  public Rotation3d getRotation3d() {
    return pigeon.getRotation3d();
  }

  public Pigeon2SimState getSimulatedState() {
    return simulationState;
  }

  public void setSimContinuousYaw(final double radians) {
    simulationState.setRawYaw(Math.toDegrees(radians));
  }

  public void setSimYawRate(final double radiansPerSecond) {
    simulationState.setAngularVelocityZ(Math.toDegrees(radiansPerSecond));
  }

  public String getName() {
    return name;
  }

  @Override
  public void close() {
    pigeon.close();
  }
}