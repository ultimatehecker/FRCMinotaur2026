package frc.robot.subsystems.intake.slam;

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
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.minolib.phoenix.PhoenixUtility;
import frc.robot.constants.IntakeConstants;

public class SlamIOHardware implements SlamIO {
    private final TalonFX motor;
    private final CANcoder encoder;

    private final TalonFXConfiguration motorConfiguration;
    private final CANcoderConfiguration encoderConfiguration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Boolean> temperatureFault;

    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final DynamicMotionMagicTorqueCurrentFOC positionTorqueCurrentRequest = new DynamicMotionMagicTorqueCurrentFOC(0.0, 0.0, 0.0).withUpdateFreqHz(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public SlamIOHardware() {
        motor = new TalonFX(IntakeConstants.kPivotMotor.getDeviceID(), IntakeConstants.kPivotMotor.getCANBus());
        encoder = new CANcoder(IntakeConstants.kPivotAbsoluteEncoder.getDeviceID(), IntakeConstants.kPivotAbsoluteEncoder.getCANBus());

        encoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(1.0)
                    .withMagnetOffset(IntakeConstants.kPivotAbsoluteEncoderOffset)
            );

        simpleTryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfiguration, 0.25));

        motorConfiguration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(IntakeConstants.kPivotMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            ).withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(120.0)
                    .withPeakReverseTorqueCurrent(-120.0)
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(IntakeConstants.kPivotMotorStatorLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(IntakeConstants.kPivotMotorSupplyLimit)
            ).withSlot0(
                new Slot0Configs()
                    .withKP(IntakeConstants.pivotkP)
                    .withKI(IntakeConstants.pivotkI)
                    .withKD(IntakeConstants.pivotkD)
                    .withKS(IntakeConstants.pivotkS)
                    .withKV(IntakeConstants.pivotkV)
                    .withKG(IntakeConstants.pivotkG)
                    .withKA(IntakeConstants.pivotkA)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
            ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withTorqueClosedLoopRampPeriod(0.02)
                    .withVoltageClosedLoopRampPeriod(0.02)
                    .withDutyCycleClosedLoopRampPeriod(0.02)
            ).withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(IntakeConstants.kPivotMaximumRotationalVelocity.in(RotationsPerSecond))
                    .withMotionMagicAcceleration(IntakeConstants.kPivotMaximumRotationalAcceleration.in(RotationsPerSecondPerSecond))
                    .withMotionMagicJerk(IntakeConstants.kPivotMaximumRotationalJerk)
            ).withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IntakeConstants.kPivotMotorReduction)
            ).withAudio(
                new AudioConfigs()
                    .withBeepOnBoot(false)   
                    .withBeepOnConfig(false)
            );

        simpleTryUntilOk(5, () -> motor.getConfigurator().apply(motorConfiguration, 0.25));
        simpleTryUntilOk(5, () -> motor.setPosition(IntakeConstants.kIntakeMinimumPosition.in(Rotations)));

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
        simpleTryUntilOk(5, () -> encoder.optimizeBusUtilization(0.0, 1.0));

        PhoenixUtility.registerSignals(
            false, 
            position, 
            velocity,
            appliedVoltage,
            torqueCurrent,
            supplyCurrent,
            temperature,
            temperatureFault
        );
    }

    @Override
    public void updateInputs(SlamIOInputs inputs) {
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
        motor.setPosition(IntakeConstants.kIntakeMinimumPosition.in(Rotations), 0.0);
    }

    @Override
    public void setPosition(double positionRadians) {
        motor.setControl(
            positionTorqueCurrentRequest
                .withPosition(Units.radiansToRotations(positionRadians))
                .withOverrideCoastDurNeutral(false)
                .withFeedForward(0.0)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kS, double kV, double kG, double kA) {
        motorConfiguration.Slot0.kP = kP;
        motorConfiguration.Slot0.kI = kI;
        motorConfiguration.Slot0.kD = kD;
        motorConfiguration.Slot0.kS = kS;
        motorConfiguration.Slot0.kV = kV;
        motorConfiguration.Slot0.kG = kG;
        motorConfiguration.Slot0.kA = kA;

        simpleTryUntilOk(5, () -> motor.getConfigurator().apply(motorConfiguration));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            motor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
    }
}
