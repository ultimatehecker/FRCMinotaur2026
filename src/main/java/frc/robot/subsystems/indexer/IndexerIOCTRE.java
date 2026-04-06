package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.minolib.phoenix.PhoenixUtility.simpleTryUntilOk;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.minolib.rev.REVUtility;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IndexerIOCTRE implements IndexerIO {
    private TalonFX indexerMotor;
    private TalonFXConfiguration indexerMotorConfiguration;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;

    private final NeutralOut neutralRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0);

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public IndexerIOCTRE() {
        indexerMotor = new TalonFX(54, "rio");

        indexerMotorConfiguration = new TalonFXConfiguration();
        indexerMotorConfiguration.MotorOutput.Inverted = IndexerConstants.kIndexerMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        indexerMotorConfiguration.Feedback.RotorToSensorRatio = IndexerConstants.kIndexerMotorReduction * 2 * Math.PI;
        indexerMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerMotorConfiguration.CurrentLimits.SupplyCurrentLimit = IndexerConstants.kIndexerMotorSupplyLimit.in(Amps);
        indexerMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerMotorConfiguration.CurrentLimits.StatorCurrentLimit = 90;

        simpleTryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerMotorConfiguration));
        simpleTryUntilOk(5, () -> indexerMotor.optimizeBusUtilization(0, 1.0));
    
        position = indexerMotor.getPosition();
        velocity = indexerMotor.getVelocity();
        acceleration = indexerMotor.getAcceleration();
        appliedVoltage = indexerMotor.getMotorVoltage();
        supplyCurrent = indexerMotor.getSupplyCurrent();
        temperature = indexerMotor.getDeviceTemp();

        simpleTryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            position, 
            velocity,
            acceleration,
            appliedVoltage,
            supplyCurrent,
            temperature
        ));
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        REVUtility.sparkStickyFault = false;

        inputs.position = position.getValue().in(Radians);
        inputs.velocity = velocity.getValue().in(RadiansPerSecond);
        inputs.acceleration = acceleration.getValue().in(RadiansPerSecondPerSecond);
        inputs.appliedVoltage = appliedVoltage.getValue().in(Volts);
        inputs.supplyCurrentAmperes = supplyCurrent.getValue().in(Amps);
        inputs.tempuratureCelcius = temperature.getValue().in(Celsius);
        inputs.isMotorConnected = BaseStatusSignal.isAllGood(
            position, 
            velocity,
            acceleration,
            appliedVoltage,
            supplyCurrent,
            temperature
        );
    }

    @Override
    public void setVoltage(double voltage) {
        indexerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setVelocity(double velocity, double feedforward) {
        indexerMotor.setControl(velocityVoltage.withVelocity(velocity).withFeedForward(feedforward));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        indexerMotorConfiguration.Slot0
            .withKP(kP)
            .withKI(kI)
            .withKD(kD);

        simpleTryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerMotorConfiguration));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // Change the neutral mode configuration in a separate thread since changing the configuration of a REV device may take a significant amount of time (~200 ms).
        brakeModeExecutor.execute(() -> {
            indexerMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            position,
            velocity,
            acceleration,
            appliedVoltage,
            supplyCurrent,
            temperature
        );
    }
}
