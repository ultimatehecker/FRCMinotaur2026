package frc.robot.subsystems.indexer;

import static frc.minolib.rev.REVUtility.tryUntilOk;
import static frc.minolib.rev.REVUtility.ifOkOrDefault;

import static edu.wpi.first.units.Units.Amps;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.minolib.hardware.CANDeviceID;
import frc.minolib.rev.REVUtility;
import frc.robot.constants.IndexerConstants;

public class IndexerIOHardware implements IndexerIO {
    private SparkMax indexerMotor;
    
    private RelativeEncoder indexerEncoder;
    private SparkClosedLoopController indexerPIDController;

    private SparkBaseConfig indexerConfiguration;

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public IndexerIOHardware() {
        indexerMotor = new SparkMax(16, MotorType.kBrushless);
        indexerEncoder = indexerMotor.getEncoder();
        indexerPIDController = indexerMotor.getClosedLoopController();

        indexerConfiguration = new SparkMaxConfig()
            .inverted(IndexerConstants.kIndexerMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) IndexerConstants.kIndexerMotorSupplyLimit.in(Amps), 50)
            .voltageCompensation(12.0);

        indexerConfiguration.encoder
            .positionConversionFactor(IndexerConstants.kIndexerMotorPositionConversionFactor)
            .velocityConversionFactor(IndexerConstants.kIndexerMotorVelocityConversionFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        indexerConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        indexerConfiguration.closedLoop
            .p(0)
            .i(0)
            .d(0);

        tryUntilOk(indexerMotor, 5, () -> indexerMotor.configure(indexerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        REVUtility.sparkStickyFault = false;

        inputs.position = indexerEncoder.getPosition();
        inputs.velocity = indexerEncoder.getVelocity();
        inputs.appliedVoltage = indexerMotor.getAppliedOutput();
        inputs.supplyCurrentAmperes = indexerMotor.getOutputCurrent();
        inputs.tempuratureCelcius = indexerMotor.getMotorTemperature();
        inputs.isMotorConnected = !REVUtility.sparkStickyFault;
    }

    @Override
    public void setVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocity, double feedforward) {
        indexerPIDController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        indexerConfiguration.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);

        tryUntilOk(indexerMotor, 5, () -> indexerMotor.configure(indexerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // Change the neutral mode configuration in a separate thread since changing the configuration of a REV device may take a significant amount of time (~200 ms).
        brakeModeExecutor.execute(() -> {
            indexerMotor.configure(
                indexerConfiguration.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast), 
                ResetMode.kNoResetSafeParameters, 
                PersistMode.kNoPersistParameters
            );
        });
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void refreshData() {
        
    }
}
