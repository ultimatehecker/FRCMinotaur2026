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
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
import frc.minolib.rev.REVUtility;
import frc.robot.RobotState;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    //private CANcoder pivotAbsoluteEncoder;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<AngularAcceleration> rollerAcceleration;
    private final StatusSignal<Voltage> rollerAppliedVoltage;
    private final StatusSignal<Current> rollerSupplyCurrent;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerTemperature;

    //private final StatusSignal<Angle> absoluteEncoderPosition;
    //private final StatusSignal<AngularVelocity> absoluteEncoderVelocity;

    private final NeutralOut neutralRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotPositionController;

    private final TalonFXConfiguration rollerMotorConfiguration;
    private final SparkBaseConfig pivotMotorConfiguration;
    //private final CANcoderConfiguration pivotAbsoluteEncoderConfiguration;

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public IntakeIOHardware() {
        rollerMotor = new TalonFX(15, GlobalConstants.kRioBus.getParent());

        rollerMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(IntakeConstants.kRollerMotorSupplyLimit.in(Amps))
                    .withSupplyCurrentLowerLimit(25)
                    .withSupplyCurrentLowerTime(1)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(IntakeConstants.kRollerMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(IntakeConstants.kRollerMotorReduction * 2 * Math.PI)
                    .withVelocityFilterTimeConstant(IntakeConstants.kRollerVelocityFilterTimeConstant)
            );

        simpleTryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerMotorConfiguration));
        simpleTryUntilOk(5, () -> rollerMotor.optimizeBusUtilization(0, 1.0));

        //pivotAbsoluteEncoder = new CANcoder(22, GlobalConstants.kRioBus.getParent());

        //pivotAbsoluteEncoderConfiguration = new CANcoderConfiguration();
        //pivotAbsoluteEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        //pivotAbsoluteEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        //pivotAbsoluteEncoderConfiguration.MagnetSensor.MagnetOffset = IntakeConstants.kPivotAbsoluteEncoderOffset.in(Rotations);    

        //simpleTryUntilOk(5, () -> pivotAbsoluteEncoder.getConfigurator().apply(pivotAbsoluteEncoderConfiguration));
        //simpleTryUntilOk(5, () -> pivotAbsoluteEncoder.optimizeBusUtilization(0, 1.0));

        pivotMotor = new TalonFX(45, GlobalConstants.kRioBus.getParent());;

        rollerMotorConfiguration = new TalonFXConfiguration()
             .withCurrentLimits(
                new CurrentLimitsConfigs()
                    //.withStatorCurrentLimitEnable(true)
                    //.withStatorCurrentLimit(100)
                    //.withSupplyCurrentLimitEnable(true)
                    //.withSupplyCurrentLimit(IntakeConstants.kRollerMotorSupplyLimit.in(Amps))
                    //.withSupplyCurrentLowerLimit(25)
                    //.withSupplyCurrentLowerTime(1)
            )
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(IntakeConstants.kPivotMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(IntakeConstants.kPivotMotorReduction * 2 * Math.PI)
            );

        simpleTryUntilOk(5, () -> pivotMotor.getConfigurator().apply(rollerMotorConfiguration));
        simpleTryUntilOk(5, () -> pivotMotor.optimizeBusUtilization(0, 1.0));
        
        pivotEncoder = pivotMotor.getEncoder();
        pivotPositionController = pivotMotor.getClosedLoopController();

        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAcceleration = rollerMotor.getAcceleration();
        rollerAppliedVoltage = rollerMotor.getMotorVoltage();
        rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
        rollerTorqueCurrent = rollerMotor.getTorqueCurrent();
        rollerTemperature = rollerMotor.getDeviceTemp();

        //absoluteEncoderPosition = pivotAbsoluteEncoder.getAbsolutePosition();
        //absoluteEncoderVelocity = pivotAbsoluteEncoder.getVelocity();

        simpleTryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            rollerPosition, 
            rollerVelocity,
            rollerAcceleration,
            rollerAppliedVoltage,
            rollerSupplyCurrent,
            rollerTorqueCurrent,
            rollerTemperature
            //absoluteEncoderPosition,
            //absoluteEncoderVelocity
        ));
        
        tryUntilOk(pivotMotor, 5, () -> pivotEncoder.setPosition(IntakeConstants.kIntakeMinimumPosition.in(Radians)));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        REVUtility.sparkStickyFault = false;

        inputs.rollerPosition = rollerPosition.getValue().in(Radians);
        inputs.rollerVelocity = rollerVelocity.getValue().in(RadiansPerSecond);
        inputs.rollerAcceleration = rollerAcceleration.getValue().in(RadiansPerSecondPerSecond);
        inputs.rollerAppliedVoltage = rollerAppliedVoltage.getValue().in(Volts);
        inputs.rollerSupplyCurrentAmperes = rollerSupplyCurrent.getValue().in(Amps);
        inputs.rollerTorqueCurrentAmperes = rollerTorqueCurrent.getValue().in(Amps);
        inputs.rollerTemperatureCelsius = rollerTemperature.getValue().in(Celsius);
        inputs.rollerMotorConnected = BaseStatusSignal.isAllGood(
            rollerPosition, 
            rollerVelocity,
            rollerAcceleration,
            rollerAppliedVoltage,
            rollerSupplyCurrent,
            rollerTorqueCurrent,
            rollerTemperature
        );

        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder.getVelocity();
        inputs.pivotAppliedVoltage = pivotMotor.getAppliedOutput();
        inputs.pivotSupplyCurrentAmperes = pivotMotor.getOutputCurrent();
        inputs.pivotMotorTempuratureCelcius = pivotMotor.getMotorTemperature();

        //inputs.pivotAbsoluteEncoderVelocity = absoluteEncoderVelocity.getValue().in(RadiansPerSecond);
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
            rollerTemperature
            //absoluteEncoderPosition
        );
    }
}