package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.minolib.phoenix.PhoenixUtility.simpleTryUntilOk;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.HoodConstants;

public class HoodIOHardware implements HoodIO {
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfiguration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> temperatureFault;

    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public HoodIOHardware() {
        motor = new TalonFX(25, GlobalConstants.kRioBus.getParent());

        motorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(HoodConstants.kMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            ).withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(120.0)
                    .withPeakReverseTorqueCurrent(-120.0)
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(HoodConstants.kMotorStatorLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(HoodConstants.kMotorSupplyLimit)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(HoodConstants.kMotorReduction)
            );

        simpleTryUntilOk(5, () -> motor.getConfigurator().apply(motorConfiguration, 0.25));
        simpleTryUntilOk(5, () -> motor.setPosition(HoodConstants.kHoodMinimumPosition.in(Rotations)));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        torqueCurrent = motor.getTorqueCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        temperature = motor.getDeviceTemp();
        temperatureFault = motor.getFault_DeviceTemp();

        simpleTryUntilOk(5, () -> torqueCurrent.setUpdateFrequency(100.0)); // TODO: Attempt to run these signals at 250Hz
        simpleTryUntilOk(
            5, () -> 
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, 
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                temperature,
                temperatureFault
            )
        );

        motor.optimizeBusUtilization(0.0, 1.0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.isMotorConnected = BaseStatusSignal.isAllGood(position, velocity, appliedVoltage, supplyCurrent, temperature);
        inputs.positionRadians = position.getValue().in(Radians);
        inputs.velocityRadiansPerSecond = velocity.getValue().in(RadiansPerSecond);
        inputs.appliedVoltage = appliedVoltage.getValue().in(Volts);
        inputs.torqueCurrentAmperes = torqueCurrent.getValue().in(Amps);
        inputs.supplyCurrentAmperes = supplyCurrent.getValue().in(Amps);
        inputs.temperatureCelsius = temperature.getValue().in(Celsius);
        inputs.temperatureFault = temperatureFault.getValue();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setOL(double amperes) {
        motor.setControl(torqueCurrentRequest.withOutput(amperes));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPosition(double positionRadians, double feedforward) {
        motor.setControl(
            positionTorqueCurrentRequest
                .withPosition(Units.radiansToRotations(positionRadians))
                .withFeedForward(feedforward)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        motorConfiguration.Slot0.kP = kP;
        motorConfiguration.Slot0.kI = kI;
        motorConfiguration.Slot0.kD = kD;

        simpleTryUntilOk(5, () -> motor.getConfigurator().apply(motorConfiguration));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            motor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            position, 
            velocity,
            appliedVoltage,
            torqueCurrent,
            supplyCurrent,
            temperature,
            temperatureFault
        );
    }
}