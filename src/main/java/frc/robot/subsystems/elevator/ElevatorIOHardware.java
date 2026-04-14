package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.minolib.phoenix.PhoenixUtility.simpleTryUntilOk;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GlobalConstants;

public class ElevatorIOHardware implements ElevatorIO {
    private final TalonFX motor;
    private final TalonFXConfiguration configuration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> temperatureFault;

    private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public ElevatorIOHardware() {
        motor = new TalonFX(ElevatorConstants.kMotor.getDeviceID(), ElevatorConstants.kMotor.getCANBus());
        configuration = new TalonFXConfiguration();

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
    public void updateInputs(ElevatorIOInputs inputs) {
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
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setOL(double amperes) {
        motor.setControl(torqueCurrentFOC.withOutput(amperes));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPosition(double position, double feedforward) {
        motor.setControl(
            positionRequest
                .withPosition(position)
                .withFeedForward(feedforward)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        configuration.Slot0.kP = kP;
        configuration.Slot0.kI = kI;
        configuration.Slot0.kD = kD;

        simpleTryUntilOk(5, () -> motor.getConfigurator().apply(configuration));
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
            //absoluteEncoderPosition
        );
    }
}
