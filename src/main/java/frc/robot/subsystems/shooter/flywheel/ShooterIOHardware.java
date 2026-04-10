<<<<<<< HEAD
package frc.robot.subsystems.shooter.flywheel;

import static frc.minolib.phoenix.PhoenixUtility.simpleTryUntilOk;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.gson.internal.TroubleshootingGuide;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;

public class ShooterIOHardware implements ShooterIO {
    public TalonFX firstShooterMotor;
    public TalonFX secondShooterMotor;
    public TalonFX thirdShooterMotor;

    private final StatusSignal<Angle> firstShooterPosition;
    private final StatusSignal<AngularVelocity> firstShooterVelocity;
    private final StatusSignal<Voltage> firstShooterAppliedVoltage;
    private final StatusSignal<Current> firstShooterSupplyCurrent;
    private final StatusSignal<Current> firstShooterTorqueCurrent;
    private final StatusSignal<Temperature> firstShooterTemperature;

    private final StatusSignal<Current> secondShooterSupplyCurrent;
    private final StatusSignal<Current> secondShooterTorqueCurrent;
    private final StatusSignal<Temperature> secondShooterTemperature;

    private final StatusSignal<Current> thirdShooterSupplyCurrent;
    private final StatusSignal<Current> thirdShooterTorqueCurrent;
    private final StatusSignal<Temperature> thirdShooterTemperature;

    private final NeutralOut neutralRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);

    private final TalonFXConfiguration primaryMotorConfiguration;
    private final TalonFXConfiguration otherMotorConfiguration;

    private final Slot0Configs shooterClosedLoopConfiguration;

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public ShooterIOHardware() {
        firstShooterMotor = new TalonFX(19);
        secondShooterMotor = new TalonFX(20);
        thirdShooterMotor = new TalonFX(21);

        primaryMotorConfiguration = new TalonFXConfiguration();
        primaryMotorConfiguration.MotorOutput.Inverted = ShooterConstants.kPrimaryShooterMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        primaryMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        primaryMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        primaryMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        primaryMotorConfiguration.CurrentLimits.StatorCurrentLimit = ShooterConstants.kShooterMotorStatorLimit.in(Amps);
        primaryMotorConfiguration.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterMotorSupplyLimit.in(Amps);
        primaryMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        otherMotorConfiguration = new TalonFXConfiguration();
        otherMotorConfiguration.MotorOutput.Inverted = ShooterConstants.kOtherShooterMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        otherMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        otherMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        otherMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        otherMotorConfiguration.CurrentLimits.StatorCurrentLimit = ShooterConstants.kShooterMotorStatorLimit.in(Amps);
        otherMotorConfiguration.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterMotorSupplyLimit.in(Amps);
        otherMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        shooterClosedLoopConfiguration = new Slot0Configs()
            .withKP(ShooterConstants.kP)
            .withKD(ShooterConstants.kD)
            .withKS(ShooterConstants.kS)
            .withKV(ShooterConstants.kV)
            .withKA(ShooterConstants.kA);

        simpleTryUntilOk(5, () -> firstShooterMotor.getConfigurator().apply(primaryMotorConfiguration, 1.0));
        simpleTryUntilOk(5, () -> secondShooterMotor.getConfigurator().apply(otherMotorConfiguration, 1.0));
        simpleTryUntilOk(5, () -> thirdShooterMotor.getConfigurator().apply(otherMotorConfiguration, 1.0));

        simpleTryUntilOk(5, () -> firstShooterMotor.getConfigurator().apply(shooterClosedLoopConfiguration, 1.0));
        simpleTryUntilOk(5, () -> secondShooterMotor.getConfigurator().apply(shooterClosedLoopConfiguration, 1.0));
        simpleTryUntilOk(5, () -> thirdShooterMotor.getConfigurator().apply(shooterClosedLoopConfiguration, 1.0));

        //secondShooterMotor.setControl(new Follower(firstShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));  
        //thirdShooterMotor.setControl(new Follower(firstShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));  

        firstShooterPosition = firstShooterMotor.getPosition();
        firstShooterVelocity = firstShooterMotor.getVelocity();
        firstShooterAppliedVoltage = firstShooterMotor.getMotorVoltage();
        firstShooterSupplyCurrent = firstShooterMotor.getSupplyCurrent();
        firstShooterTorqueCurrent = firstShooterMotor.getTorqueCurrent();
        firstShooterTemperature = firstShooterMotor.getDeviceTemp();

        secondShooterSupplyCurrent = secondShooterMotor.getSupplyCurrent();
        secondShooterTorqueCurrent = secondShooterMotor.getTorqueCurrent();
        secondShooterTemperature = secondShooterMotor.getDeviceTemp();

        thirdShooterSupplyCurrent = thirdShooterMotor.getSupplyCurrent();
        thirdShooterTorqueCurrent = thirdShooterMotor.getTorqueCurrent();
        thirdShooterTemperature = thirdShooterMotor.getDeviceTemp();

        simpleTryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            firstShooterPosition,
            firstShooterVelocity,
            firstShooterAppliedVoltage,
            firstShooterSupplyCurrent,
            firstShooterTorqueCurrent,
            firstShooterTemperature,
            secondShooterSupplyCurrent,
            secondShooterTorqueCurrent,
            secondShooterTemperature,
            thirdShooterSupplyCurrent,
            thirdShooterTorqueCurrent,
            thirdShooterTemperature
        ));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.firstShooterPosition = firstShooterPosition.getValue().in(Radians);
        inputs.firstShooterVelocity = firstShooterVelocity.getValue().in(RadiansPerSecond);
        inputs.firstShooterAppliedVoltage = firstShooterAppliedVoltage.getValue().in(Volts);
        inputs.firstShooterSupplyCurrentAmperes = firstShooterSupplyCurrent.getValue().in(Amps);
        inputs.firstShooterTorqueCurrentAmperes = firstShooterTorqueCurrent.getValue().in(Amps);
        inputs.firstShooterTempuratureCelcius = firstShooterTemperature.getValue().in(Celsius);
        inputs.isFirstShooterConnected = BaseStatusSignal.isAllGood(
            firstShooterPosition,
            firstShooterVelocity,
            firstShooterAppliedVoltage,
            firstShooterSupplyCurrent,
            firstShooterTorqueCurrent,
            firstShooterTemperature
        );

        inputs.secondShooterSupplyCurrentAmperes = secondShooterSupplyCurrent.getValue().in(Amps);
        inputs.secondShooterTorqueCurrentAmperes = secondShooterTorqueCurrent.getValue().in(Amps);
        inputs.secondShooterTempuratureCelcius = secondShooterTemperature.getValue().in(Celsius);
        inputs.isSecondShooterConnected = BaseStatusSignal.isAllGood(
            secondShooterSupplyCurrent,
            secondShooterTorqueCurrent,
            secondShooterTemperature
        );

        inputs.thirdShooterSupplyCurrentAmperes = thirdShooterSupplyCurrent.getValue().in(Amps);
        inputs.thirdShooterTorqueCurrentAmperes = thirdShooterTorqueCurrent.getValue().in(Amps);
        inputs.thirdShooterTempuratureCelcius = thirdShooterTemperature.getValue().in(Celsius);
        inputs.isThirdShooterConnected = BaseStatusSignal.isAllGood(
            thirdShooterSupplyCurrent,
            thirdShooterTorqueCurrent,
            thirdShooterTemperature
        );
    }

    @Override
    public void setVoltage(double voltage) {
        firstShooterMotor.setControl(voltageRequest.withOutput(voltage));
        secondShooterMotor.setControl(voltageRequest.withOutput(voltage));
        thirdShooterMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setVelocity(double velocity, double feedforward) {
        firstShooterMotor.setControl(velocityRequest.withVelocity(velocity).withFeedForward(feedforward));
        secondShooterMotor.setControl(velocityRequest.withVelocity(velocity).withFeedForward(feedforward));
        thirdShooterMotor.setControl(velocityRequest.withVelocity(velocity).withFeedForward(feedforward));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        shooterClosedLoopConfiguration.kP = kP;
        shooterClosedLoopConfiguration.kI = kI;
        shooterClosedLoopConfiguration.kD = kD;

        simpleTryUntilOk(5, () -> firstShooterMotor.getConfigurator().apply(shooterClosedLoopConfiguration, 1.0));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            firstShooterMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            secondShooterMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            thirdShooterMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
    }

    @Override
    public void stop() {
        firstShooterMotor.setControl(neutralRequest);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            firstShooterPosition,
            firstShooterVelocity,
            firstShooterAppliedVoltage,
            firstShooterSupplyCurrent,
            firstShooterTorqueCurrent,
            firstShooterTemperature,
            secondShooterSupplyCurrent,
            secondShooterTorqueCurrent,
            secondShooterTemperature,
            thirdShooterSupplyCurrent,
            thirdShooterTorqueCurrent,
            thirdShooterTemperature
        );
    }
}
=======
package frc.robot.subsystems.shooter.flywheel;

import static frc.minolib.phoenix.PhoenixUtility.simpleTryUntilOk;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterIOHardware implements ShooterIO {
    public final TalonFX firstShooterMotor;
    public final TalonFX secondShooterMotor;
    public final TalonFX thirdShooterMotor;
    public final TalonFX fourthShooterMotor;

    private final TalonFXConfiguration configuration;

    private final StatusSignal<Angle> firstShooterPosition;
    private final StatusSignal<AngularVelocity> firstShooterVelocity;
    private final StatusSignal<Voltage> firstShooterAppliedVoltage;
    private final StatusSignal<Current> firstShooterSupplyCurrent;
    private final StatusSignal<Current> firstShooterTorqueCurrent;
    private final StatusSignal<Temperature> firstShooterTemperature;
    private final StatusSignal<Boolean> firstShooterTemperatureFault;

    private final StatusSignal<Current> secondShooterSupplyCurrent;
    private final StatusSignal<Temperature> secondShooterTemperature;
    private final StatusSignal<Boolean> secondhooterTemperatureFault;

    private final StatusSignal<Current> thirdShooterSupplyCurrent;
    private final StatusSignal<Temperature> thirdShooterTemperature;
    private final StatusSignal<Boolean> thirdShooterTemperatureFault;

    private final StatusSignal<Current> fourthShooterSupplyCurrent;
    private final StatusSignal<Temperature> fourthShooterTemperature;
    private final StatusSignal<Boolean> fourthShooterTemperatureFault;

    private final NeutralOut neutralRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public ShooterIOHardware() {
        firstShooterMotor = new TalonFX(21, GlobalConstants.kRioBus.getParent());
        secondShooterMotor = new TalonFX(22, GlobalConstants.kRioBus.getParent());
        thirdShooterMotor = new TalonFX(23, GlobalConstants.kRioBus.getParent());
        fourthShooterMotor = new TalonFX(24, GlobalConstants.kRioBus.getParent());

        secondShooterMotor.setControl(new Follower(firstShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));  
        thirdShooterMotor.setControl(new Follower(firstShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));  
        fourthShooterMotor.setControl(new Follower(firstShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));  

        configuration = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(ShooterConstants.kPrimaryShooterMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ShooterConstants.kShooterMotorStatorLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ShooterConstants.kShooterMotorSupplyLimit)
            ).withSlot0(
                new Slot0Configs()
                    .withKP(ShooterConstants.kP)
                    .withKD(ShooterConstants.kD)
                    .withKS(ShooterConstants.kS)
                    .withKV(ShooterConstants.kV)
                    .withKA(ShooterConstants.kA)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(ShooterConstants.kShooterMotorReduction)
            );

        simpleTryUntilOk(5, () -> firstShooterMotor.getConfigurator().apply(configuration, 1.0));
        simpleTryUntilOk(5, () -> secondShooterMotor.getConfigurator().apply(new TalonFXConfiguration(), 1.0));
        simpleTryUntilOk(5, () -> thirdShooterMotor.getConfigurator().apply(new TalonFXConfiguration(), 1.0));
        simpleTryUntilOk(5, () -> fourthShooterMotor.getConfigurator().apply(new TalonFXConfiguration(), 1.0));

        firstShooterPosition = firstShooterMotor.getPosition();
        firstShooterVelocity = firstShooterMotor.getVelocity();
        firstShooterAppliedVoltage = firstShooterMotor.getMotorVoltage();
        firstShooterSupplyCurrent = firstShooterMotor.getSupplyCurrent();
        firstShooterTorqueCurrent = firstShooterMotor.getTorqueCurrent();
        firstShooterTemperature = firstShooterMotor.getDeviceTemp();
        firstShooterTemperatureFault = firstShooterMotor.getFault_DeviceTemp();

        secondShooterSupplyCurrent = secondShooterMotor.getSupplyCurrent();
        secondShooterTemperature = secondShooterMotor.getDeviceTemp();
        secondhooterTemperatureFault = firstShooterMotor.getFault_DeviceTemp();

        thirdShooterSupplyCurrent = thirdShooterMotor.getSupplyCurrent();
        thirdShooterTemperature = thirdShooterMotor.getDeviceTemp();
        thirdShooterTemperatureFault = firstShooterMotor.getFault_DeviceTemp();

        fourthShooterSupplyCurrent = fourthShooterMotor.getSupplyCurrent();
        fourthShooterTemperature = fourthShooterMotor.getDeviceTemp();
        fourthShooterTemperatureFault = fourthShooterMotor.getFault_DeviceTemp();

        simpleTryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            firstShooterPosition,
            firstShooterVelocity,
            firstShooterAppliedVoltage,
            firstShooterSupplyCurrent,
            firstShooterTorqueCurrent,
            firstShooterTemperature,
            secondShooterSupplyCurrent,
            secondShooterTemperature,
            secondhooterTemperatureFault,
            thirdShooterSupplyCurrent,
            thirdShooterTemperature,
            thirdShooterTemperatureFault,
            fourthShooterSupplyCurrent,
            fourthShooterTemperature,
            fourthShooterTemperatureFault
        ));

        firstShooterMotor.optimizeBusUtilization(0.0, 1.0);
        secondShooterMotor.optimizeBusUtilization(0.0, 1.0);
        thirdShooterMotor.optimizeBusUtilization(0.0, 1.0);
        fourthShooterMotor.optimizeBusUtilization(0.0, 1.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.firstShooterPosition = firstShooterPosition.getValue().in(Rotations);
        inputs.firstShooterVelocity = firstShooterVelocity.getValue().in(RotationsPerSecond) * 60;
        inputs.firstShooterAppliedVoltage = firstShooterAppliedVoltage.getValue().in(Volts);
        inputs.firstShooterSupplyCurrentAmperes = firstShooterSupplyCurrent.getValue().in(Amps);
        inputs.firstShooterTorqueCurrentAmperes = firstShooterTorqueCurrent.getValue().in(Amps);
        inputs.firstShooterTempuratureCelcius = firstShooterTemperature.getValue().in(Celsius);
        inputs.isFirstShooterConnected = BaseStatusSignal.isAllGood(
            firstShooterPosition,
            firstShooterVelocity,
            firstShooterAppliedVoltage,
            firstShooterSupplyCurrent,
            firstShooterTorqueCurrent,
            firstShooterTemperature,
            firstShooterTemperatureFault
        );

        inputs.secondShooterSupplyCurrentAmperes = secondShooterSupplyCurrent.getValue().in(Amps);
        inputs.secondShooterTempuratureCelcius = secondShooterTemperature.getValue().in(Celsius);
        inputs.isSecondShooterConnected = BaseStatusSignal.isAllGood(
            secondShooterSupplyCurrent,
            secondShooterTemperature,
            secondhooterTemperatureFault
        );

        inputs.thirdShooterSupplyCurrentAmperes = thirdShooterSupplyCurrent.getValue().in(Amps);
        inputs.thirdShooterTempuratureCelcius = thirdShooterTemperature.getValue().in(Celsius);
        inputs.isThirdShooterConnected = BaseStatusSignal.isAllGood(
            thirdShooterSupplyCurrent,
            thirdShooterTemperature,
            thirdShooterTemperatureFault
        );

        inputs.fourthShooterSupplyCurrentAmperes = fourthShooterSupplyCurrent.getValue().in(Amps);
        inputs.fourthShooterTempuratureCelcius = fourthShooterTemperature.getValue().in(Celsius);
        inputs.isFourthShooterConnected = BaseStatusSignal.isAllGood(
            fourthShooterSupplyCurrent,
            fourthShooterTemperature,
            fourthShooterTemperatureFault
        );
    }

    @Override
    public void setVoltage(double voltage) {
        firstShooterMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setOL(double amperes) {
        firstShooterMotor.setControl(torqueCurrentRequest.withOutput(amperes));
    }

    @Override
    public void stop() {
        firstShooterMotor.setControl(neutralRequest);
    }

    @Override
    public void setVelocity(double velocity, double feedforward) {
        firstShooterMotor.setControl(
            velocityRequest
                .withVelocity(velocity / 60)
                .withFeedForward(feedforward)
                .withEnableFOC(true)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        configuration.Slot0.kP = kP;
        configuration.Slot0.kI = kI;
        configuration.Slot0.kD = kD;

        simpleTryUntilOk(5, () -> firstShooterMotor.getConfigurator().apply(configuration, 1.0));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            firstShooterMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            firstShooterPosition,
            firstShooterVelocity,
            firstShooterAppliedVoltage,
            firstShooterSupplyCurrent,
            firstShooterTorqueCurrent,
            firstShooterTemperature,
            firstShooterTemperatureFault,
            secondShooterSupplyCurrent,
            secondShooterTemperature,
            secondhooterTemperatureFault,
            thirdShooterSupplyCurrent,
            thirdShooterTemperature,
            thirdShooterTemperatureFault,
            fourthShooterSupplyCurrent,
            fourthShooterTemperature,
            fourthShooterTemperatureFault
        );
    }
}
>>>>>>> 63bd9e1 (idk what changed ngl)
