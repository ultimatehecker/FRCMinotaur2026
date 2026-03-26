package frc.robot.subsystems.tower;

import static frc.minolib.rev.REVUtility.tryUntilOk;
import static frc.minolib.rev.REVUtility.ifOkOrDefault;

import static edu.wpi.first.units.Units.Amps;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.DoubleSupplier;

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

import frc.minolib.hardware.MinoCANDevice;
import frc.minolib.rev.REVUtility;
import frc.robot.constants.TowerConstants;
import frc.robot.subsystems.tower.TowerIO.TowerIOInputs;

public class TowerIOHardware implements TowerIO {
    private SparkMax topTowerMotor;
    private SparkMax bottomTowerMotor;
    
    private RelativeEncoder topTowerEncoder;
    private RelativeEncoder bottomTowerEncoder;
    private SparkClosedLoopController topTowerPIDController;
    private SparkClosedLoopController bottomTowerPIDController;

    private SparkBaseConfig topTowerMotorConfiguration;
    private SparkBaseConfig bottomTowerMotorConfiguration;

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public TowerIOHardware() {
        topTowerMotor = new SparkMax(18, MotorType.kBrushless);
        topTowerEncoder = topTowerMotor.getEncoder();
        topTowerPIDController = topTowerMotor.getClosedLoopController();

        bottomTowerMotor = new SparkMax(17, MotorType.kBrushless);
        bottomTowerEncoder = bottomTowerMotor.getEncoder();
        bottomTowerPIDController = bottomTowerMotor.getClosedLoopController();

        topTowerMotorConfiguration = new SparkMaxConfig()
            .inverted(TowerConstants.kTopRollerMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) TowerConstants.kTopRollerMotorSupplyLimit.in(Amps), (int) TowerConstants.kTopRollerMotorSupplyLimit.in(Amps))
            .voltageCompensation(12.0);

        topTowerMotorConfiguration.encoder
            .positionConversionFactor(TowerConstants.kTopRollerMotorPositionConversionFactor)
            .velocityConversionFactor(TowerConstants.kTopRollerMotorVelocityConversionFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        topTowerMotorConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        topTowerMotorConfiguration.closedLoop
            .p(0)
            .i(0)
            .d(0);

        tryUntilOk(topTowerMotor, 5, () -> topTowerMotor.configure(topTowerMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        bottomTowerMotorConfiguration = new SparkMaxConfig()
            .inverted(TowerConstants.kTopRollerMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) TowerConstants.kBottomRollerMotorSupplyLimit.in(Amps), (int) TowerConstants.kBottomRollerMotorSupplyLimit.in(Amps))
            .voltageCompensation(12.0);

        bottomTowerMotorConfiguration.encoder
            .positionConversionFactor(TowerConstants.kBottomRollerMotorPositionConversionFactor)
            .velocityConversionFactor(TowerConstants.kBottomRollerMotorVelocityConversionFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        bottomTowerMotorConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        bottomTowerMotorConfiguration.closedLoop
            .p(0)
            .i(0)
            .d(0);

        tryUntilOk(bottomTowerMotor, 5, () -> bottomTowerMotor.configure(bottomTowerMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(TowerIOInputs inputs) {
        REVUtility.sparkStickyFault = false;

        ifOkOrDefault(topTowerMotor, topTowerEncoder::getPosition, inputs.topRollerPosition);
        ifOkOrDefault(topTowerMotor, topTowerEncoder::getVelocity, inputs.topRollerVelocity);
        ifOkOrDefault(topTowerMotor, new DoubleSupplier[] { topTowerMotor::getBusVoltage, topTowerMotor::getAppliedOutput }, x -> x[0] * x[1], inputs.topRollerAppliedVoltage);
        ifOkOrDefault(topTowerMotor, topTowerMotor::getOutputCurrent, inputs.topRollerSupplyCurrentAmperes);
        ifOkOrDefault(topTowerMotor, topTowerMotor::getMotorTemperature, inputs.topRollerTempuratureCelcius);

        inputs.isTopRollerMotorConnected = !REVUtility.sparkStickyFault;
        REVUtility.sparkStickyFault = false;

        ifOkOrDefault(bottomTowerMotor, bottomTowerEncoder::getPosition, inputs.bottomRollerPosition);
        ifOkOrDefault(bottomTowerMotor, bottomTowerEncoder::getVelocity, inputs.bottomRollerVelocity);
        ifOkOrDefault(bottomTowerMotor, new DoubleSupplier[] { bottomTowerMotor::getBusVoltage, bottomTowerMotor::getAppliedOutput }, x -> x[0] * x[1], inputs.bottomRollerAppliedVoltage);
        ifOkOrDefault(bottomTowerMotor, bottomTowerMotor::getOutputCurrent, inputs.bottomRollerSupplyCurrentAmperes);
        ifOkOrDefault(bottomTowerMotor, bottomTowerMotor::getMotorTemperature, inputs.bottomRollerTempuratureCelcius);

        inputs.isBottomRollerMotorConnected = !REVUtility.sparkStickyFault;
    }

    @Override
    public void setTopRollerVoltage(double voltage) {
        topTowerMotor.setVoltage(voltage);
    }

    @Override
    public void setBottomRollerVoltage(double voltage) {
        bottomTowerMotor.setVoltage(voltage);
    }

    @Override
    public void setTopRollerVelocity(double velocity, double feedforward) {
        topTowerPIDController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void setBottomRollerVelocity(double velocity, double feedforward) {
        bottomTowerPIDController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }
    @Override
    public void setTopRollerPID(double kP, double kI, double kD) {
        topTowerMotorConfiguration.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);

        tryUntilOk(topTowerMotor, 5, () -> topTowerMotor.configure(topTowerMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setBottomRollerPID(double kP, double kI, double kD) {
        bottomTowerMotorConfiguration.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);

        tryUntilOk(bottomTowerMotor, 5, () -> bottomTowerMotor.configure(bottomTowerMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            topTowerMotor.configure(
                topTowerMotorConfiguration.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast), 
                ResetMode.kNoResetSafeParameters, 
                PersistMode.kNoPersistParameters
            );

            bottomTowerMotor.configure(
                bottomTowerMotorConfiguration.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast), 
                ResetMode.kNoResetSafeParameters, 
                PersistMode.kNoPersistParameters
            );
        });
    }

    @Override
    public void stopRollers() {
        setTopRollerVoltage(0.0);
        setBottomRollerVoltage(0.0);
    }
}
