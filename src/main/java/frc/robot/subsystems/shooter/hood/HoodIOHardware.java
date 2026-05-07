package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.minolib.phoenix.PhoenixUtility.simpleTryUntilOk;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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

import frc.minolib.phoenix.PhoenixUtility;
import frc.robot.constants.HoodConstants;

public class HoodIOHardware implements HoodIO {
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfiguration;
    private final Slot0Configs closedLoopConfiguration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> temperatureFault;

    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentRequest = new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public HoodIOHardware() {
        motor = new TalonFX(HoodConstants.kMotor.getDeviceID(), HoodConstants.kMotor.getCANBus());

        closedLoopConfiguration = new Slot0Configs();
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
            ).withSlot0(
                closedLoopConfiguration
                    .withKP(HoodConstants.kP)
                    .withKI(HoodConstants.kI)
                    .withKD(HoodConstants.kD)
                    .withKS(HoodConstants.kS)
                    .withKV(HoodConstants.kV)
                    .withKA(HoodConstants.kA)
            ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withTorqueClosedLoopRampPeriod(0.02)
                    .withVoltageClosedLoopRampPeriod(0.02)
                    .withDutyCycleClosedLoopRampPeriod(0.02)
            ).withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(HoodConstants.kMaximumRotationalVelocity.in(RotationsPerSecond))
                    .withMotionMagicAcceleration(HoodConstants.kMaximumRotationalAcceleration.in(RotationsPerSecondPerSecond))
                    .withMotionMagicJerk(HoodConstants.kMaximumRotationalJerk)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(HoodConstants.kMotorReduction)
            ).withAudio(
                new AudioConfigs()
                    .withBeepOnBoot(false)   
                    .withBeepOnConfig(false)
            );

        simpleTryUntilOk(5, () -> motor.getConfigurator().apply(motorConfiguration, 0.25));

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

        simpleTryUntilOk(5, () -> motor.optimizeBusUtilization(0.0, 1.0));

        PhoenixUtility.registerSignals(
            false, 
            position,
            velocity,
            appliedVoltage,
            supplyCurrent,
            torqueCurrent,
            temperature,
            temperatureFault
        );
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
    public void resetPosition() {
        motor.setPosition(HoodConstants.kZeoredPosition.in(Rotations), 0.0);
    }

    @Override
    public void setPosition(double positionRadians) {
        motor.setControl(
            motionMagicTorqueCurrentRequest
                .withPosition(Units.radiansToRotations(positionRadians))
                .withOverrideCoastDurNeutral(false)
                .withSlot(0)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
        closedLoopConfiguration.kP = kP;
        closedLoopConfiguration.kI = kI;
        closedLoopConfiguration.kD = kD;
        closedLoopConfiguration.kS = kS;
        closedLoopConfiguration.kV = kV;
        closedLoopConfiguration.kA = kA;

        simpleTryUntilOk(5, () -> motor.getConfigurator().apply(closedLoopConfiguration, 0.0));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            motor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
    }
}