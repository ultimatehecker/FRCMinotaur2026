package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.minolib.phoenix.PhoenixUtility.simpleTryUntilOk;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.minolib.hardware.CANDeviceID;
import frc.minolib.hardware.MinoCANDevice;
import frc.robot.subsystems.rollers.RollerSystemIO.RollerSystemIOInputs;

public class RollerSystemIOHardware implements RollerSystemIO{
    private final TalonFX motor;
    private final TalonFXConfiguration configuration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> temperatureFault;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final TorqueCurrentFOC torqueCurrentOut = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);
    private final NeutralOut neutralOut = new NeutralOut();

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);
    private static final Executor currentLimitExecutor = Executors.newFixedThreadPool(1);

    private final double reduction;

    public RollerSystemIOHardware(final MinoCANDevice device, double currentLimitAmperes, boolean inverted, boolean brakeEnabled, double reduction) {
        this.reduction = reduction;
        motor = new TalonFX(device.deviceNumber, device.CANBus.getParent());

        configuration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(brakeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)   
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(currentLimitAmperes)
            ).withFeedback(
                new FeedbackConfigs()
                    .withVelocityFilterTimeConstant(0.1)
                    .withSensorToMechanismRatio(reduction)
            );

        simpleTryUntilOk(5, () -> motor.getConfigurator().apply(configuration));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        torqueCurrent = motor.getTorqueCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        temperature = motor.getDeviceTemp();
        temperatureFault = motor.getFault_DeviceTemp();

        simpleTryUntilOk(
            5,
            () -> BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                torqueCurrent,
                supplyCurrent,
                temperature,
                temperatureFault
            )
        );

        simpleTryUntilOk(5, () -> motor.optimizeBusUtilization(0, 1.0));
    }

    @Override
    public void updateInputs(RollerSystemIOInputs inputs) {
        inputs.isMotorConnected = BaseStatusSignal.isAllGood(position, velocity, appliedVoltage, torqueCurrent, supplyCurrent, temperature, temperatureFault);
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
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setOL(double amperes) {
        motor.setControl(torqueCurrentOut.withOutput(amperes));
    }

    @Override
    public void stop() {
        motor.setControl(neutralOut);
    }

    @Override
    public void setCurrentLimit(double currentLimit) {
        brakeModeExecutor.execute(() -> {
            configuration.withCurrentLimits(configuration.CurrentLimits.withStatorCurrentLimit(currentLimit));
            simpleTryUntilOk(5, () -> motor.getConfigurator().apply(configuration));
        });
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
