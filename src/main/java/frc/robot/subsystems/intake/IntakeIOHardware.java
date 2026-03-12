package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import static frc.minolib.phoenix.PhoenixUtility.simpleTryUntilOk;
import static frc.minolib.rev.REVUtility.tryUntilOk;
import static frc.minolib.rev.REVUtility.ifOkOrDefault;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.minolib.hardware.MinoCANDevice;
import frc.robot.constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    private TalonFX rollerMotor;
    private SparkMax pivotMotor;
    private CANcoder pivotAbsoluteEncoder;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<AngularAcceleration> rollerAcceleration;
    private final StatusSignal<Voltage> rollerAppliedVoltage;
    private final StatusSignal<Current> rollerSupplyCurrent;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerTemperature;

    private final StatusSignal<Angle> absoluteEncoderPosition;

    private final NeutralOut neutralRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotPositionController;

    private final TalonFXConfiguration rollerMotorConfiguration;
    private final SparkBaseConfig pivotMotorConfiguration;
    private final CANcoderConfiguration pivotAbsoluteEncoderConfiguration;

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public IntakeIOHardware(MinoCANDevice rollerCANDevice, MinoCANDevice pivotCANDevice, MinoCANDevice pivotEncoderCANDevice) {
        rollerMotor = new TalonFX(rollerCANDevice.deviceNumber, rollerCANDevice.CANBus.getParent());

        rollerMotorConfiguration = new TalonFXConfiguration();
        rollerMotorConfiguration.MotorOutput.Inverted = IntakeConstants.kRollerMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        rollerMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerMotorConfiguration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRollerMotorSupplyLimit.in(Amps);
        rollerMotorConfiguration.Feedback.VelocityFilterTimeConstant = IntakeConstants.kRollerVelocityFilterTimeConstant;

        simpleTryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerMotorConfiguration));
        simpleTryUntilOk(5, () -> rollerMotor.optimizeBusUtilization(0, 1.0));

        pivotAbsoluteEncoder = new CANcoder(pivotEncoderCANDevice.deviceNumber, pivotEncoderCANDevice.CANBus.getParent());

        pivotAbsoluteEncoderConfiguration = new CANcoderConfiguration();
        pivotAbsoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        pivotAbsoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotAbsoluteEncoderConfiguration.MagnetSensor.MagnetOffset = IntakeConstants.kPivotAbsoluteEncoderOffset.in(Rotations);    

        simpleTryUntilOk(5, () -> pivotAbsoluteEncoder.getConfigurator().apply(pivotAbsoluteEncoderConfiguration));
        simpleTryUntilOk(5, () -> pivotAbsoluteEncoder.optimizeBusUtilization(0, 1.0));

        pivotMotor = new SparkMax(pivotCANDevice.deviceNumber, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPositionController = pivotMotor.getClosedLoopController();

        pivotMotorConfiguration = new SparkMaxConfig()
            .inverted(IntakeConstants.kPivotMotorInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit((int) IntakeConstants.kPivotMotorSupplyLimit.in(Amps), 50)
            .voltageCompensation(12.0);

        pivotMotorConfiguration.encoder
            .positionConversionFactor(IntakeConstants.kPivotMotorPositionConversionFactor)
            .velocityConversionFactor(IntakeConstants.kPivotMotorVelocityConversionFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        pivotMotorConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        pivotMotorConfiguration.closedLoop
            .p(0)
            .i(0)
            .d(0);

        tryUntilOk(pivotMotor, 5, () -> pivotMotor.configure(pivotMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(pivotMotor, 5, () -> pivotEncoder.setPosition(0.0));

        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAcceleration = rollerMotor.getAcceleration();
        rollerAppliedVoltage = rollerMotor.getMotorVoltage();
        rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
        rollerTorqueCurrent = rollerMotor.getTorqueCurrent();
        rollerTemperature = rollerMotor.getDeviceTemp();

        absoluteEncoderPosition = pivotAbsoluteEncoder.getAbsolutePosition();

        simpleTryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            rollerPosition, 
            rollerVelocity,
            rollerAcceleration,
            rollerAppliedVoltage,
            rollerSupplyCurrent,
            rollerTorqueCurrent,
            rollerTemperature,
            absoluteEncoderPosition
        ));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerPosition = rollerPosition.getValue().in(Radians);
        inputs.rollerVelocity = rollerVelocity.getValue().in(RadiansPerSecond);
        inputs.rollerAcceleration = rollerAcceleration.getValue().in(RadiansPerSecondPerSecond);
        inputs.rollerAppliedVoltage = rollerAppliedVoltage.getValue().in(Volts);
        inputs.rollerSupplyCurrentAmperes = rollerSupplyCurrent.getValue().in(Amps);
        inputs.rollerTorqueCurrentAmperes = rollerTorqueCurrent.getValue().in(Amps);
        inputs.rollerTemperatureCelsius = rollerTemperature.getValue().in(Celsius);

        ifOkOrDefault(pivotMotor, pivotEncoder::getPosition, inputs.pivotPosition);
        ifOkOrDefault(pivotMotor, pivotEncoder::getVelocity, inputs.pivotVelocity);
        ifOkOrDefault(pivotMotor, new DoubleSupplier[] { pivotMotor::getBusVoltage, pivotMotor::getAppliedOutput }, x -> x[0] * x[1], inputs.pivotAppliedVoltage);
        ifOkOrDefault(pivotMotor, pivotMotor::getOutputCurrent, inputs.pivotSupplyCurrentAmperes);
        ifOkOrDefault(pivotMotor, pivotMotor::getMotorTemperature, inputs.pivotMotorTempuratureCelcius);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setPivotPosition(double position, double feedforward) {
        pivotPositionController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void setRollerTorqueCurrent(double amperes) {
        rollerMotor.setControl(torqueCurrentRequest.withOutput(amperes));
    }

    @Override
    public void stopRollers() {
        rollerMotor.setControl(neutralRequest);
    }

    @Override
    public void setPivotPID(double kP, double kI, double kD) {
        pivotMotorConfiguration.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);

        tryUntilOk(pivotMotor, 5, () -> pivotMotor.configure(pivotMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // Change the neutral mode configuration in a separate thread since changing the configuration of a REV device may take a significant amount of time (~200 ms).
        brakeModeExecutor.execute(() -> {
            pivotMotor.configure(
                pivotMotorConfiguration.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast), 
                ResetMode.kNoResetSafeParameters, 
                PersistMode.kNoPersistParameters
            );

            rollerMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            rollerPosition, 
            rollerVelocity,
            rollerAcceleration,
            rollerAppliedVoltage,
            rollerSupplyCurrent,
            rollerTorqueCurrent,
            rollerTemperature,
            absoluteEncoderPosition
        );
    }
}