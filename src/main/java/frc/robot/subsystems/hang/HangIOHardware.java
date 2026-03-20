package frc.robot.subsystems.hang;

import static frc.minolib.rev.REVUtility.tryUntilOk;
import static frc.minolib.rev.REVUtility.ifOkOrDefault;

import static edu.wpi.first.units.Units.Amps;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.minolib.rev.REVUtility;
import frc.robot.constants.HangConstants;

public class HangIOHardware implements HangIO {
    private SparkMax pivotMotor;
    private SparkMax winderMotor;
    
    private RelativeEncoder pivotEncoder;
    private RelativeEncoder winderEncoder;
    private SparkClosedLoopController pivotPIDController;
    private SparkClosedLoopController winderPIDController;

    private SparkBaseConfig pivotMotorConfiguration;
    private SparkBaseConfig winderMotorConfiguration;

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public HangIOHardware() {
        pivotMotor = new SparkMax(24, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = pivotMotor.getClosedLoopController();

        winderMotor = new SparkMax(25, MotorType.kBrushless);
        winderEncoder = winderMotor.getEncoder();
        winderPIDController = winderMotor.getClosedLoopController();

        pivotMotorConfiguration = new SparkMaxConfig()
            .inverted(HangConstants.kPivotMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) HangConstants.kPivotMotorSupplyLimit.in(Amps), 50)
            .voltageCompensation(12.0);

        pivotMotorConfiguration.encoder
            .positionConversionFactor(HangConstants.kPivotMotorPositionConversionFactor)
            .velocityConversionFactor(HangConstants.kPivotMotorVelocityConversionFactor)
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

        tryUntilOk(pivotMotor, 5, () -> pivotMotor.configure(pivotMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

        winderMotorConfiguration = new SparkMaxConfig()
            .inverted(HangConstants.kWinderMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) HangConstants.kWinderMotorSupplyLimit.in(Amps), 50)
            .voltageCompensation(12.0);

        winderMotorConfiguration.encoder
            .positionConversionFactor(HangConstants.kWinderMotorPositionConversionFactor)
            .velocityConversionFactor(HangConstants.kWinderMotorVelocityConversionFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        winderMotorConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        winderMotorConfiguration.closedLoop
            .p(0)
            .i(0)
            .d(0);

        tryUntilOk(winderMotor, 5, () -> winderMotor.configure(winderMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void updateInputs(HangIOInputs inputs) {
        REVUtility.sparkStickyFault = false;

        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotVelocity = pivotEncoder .getVelocity();
        inputs.pivotAppliedVoltage = pivotMotor.getAppliedOutput();
        inputs.pivotSupplyCurrentAmperes = pivotMotor.getOutputCurrent();
        inputs.pivotTempuratureCelcius = pivotMotor.getMotorTemperature();

        inputs.winderPosition = winderEncoder.getPosition();
        inputs.winderVelocity = winderEncoder.getVelocity();
        inputs.winderAppliedVoltage = winderMotor.getAppliedOutput();
        inputs.winderSupplyCurrentAmperes = winderMotor.getOutputCurrent();
        inputs.winderTempuratureCelcius = winderMotor.getMotorTemperature();
    }

    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    public void setWinderVoltage(double voltage) {
        winderMotor.setVoltage(voltage);
    }

    public void setPivotPosition(double position, double feedforward) {
        pivotPIDController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    public void setWinderPosition(double position, double feedforward) {
        winderPIDController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    public void setPivotPID(double kP, double kI, double kD) {
        pivotMotorConfiguration.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);
        
        tryUntilOk(pivotMotor, 5, () -> pivotMotor.configure(pivotMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    public void setWinderPID(double kP, double kI, double kD) {
        winderMotorConfiguration.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);
        
        tryUntilOk(winderMotor, 5, () -> winderMotor.configure(winderMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            pivotMotor.configure(
                pivotMotorConfiguration.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast), 
                ResetMode.kNoResetSafeParameters, 
                PersistMode.kNoPersistParameters
            );
        });
    }

    public void stop() {
        setPivotVoltage(0.0);
        setWinderVoltage(0.0);
    }

    @Override
    public void refreshData() {

    }
}