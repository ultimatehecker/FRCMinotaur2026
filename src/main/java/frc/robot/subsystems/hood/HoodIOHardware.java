package frc.robot.subsystems.hood;

import static frc.minolib.rev.REVUtility.tryUntilOk;
import static frc.minolib.rev.REVUtility.ifOkOrDefault;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.minolib.hardware.MinoCANDevice;
import frc.minolib.rev.REVUtility;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

public class HoodIOHardware implements HoodIO {
    private SparkMax hoodMotor;

    private RelativeEncoder hoodEncoder;
    private SparkClosedLoopController hoodPositionController;

    private SparkBaseConfig hoodMotorConfiguration;

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public HoodIOHardware() {
        hoodMotor = new SparkMax(22, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
        hoodPositionController = hoodMotor.getClosedLoopController();

        hoodMotorConfiguration = new SparkMaxConfig()
            .inverted(HoodConstants.kHoodMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) HoodConstants.kHoodMotorSupplyLimit.in(Amps), 50)
            .voltageCompensation(12.0);

        hoodMotorConfiguration.encoder
            .positionConversionFactor(HoodConstants.kHoodMotorPositionConversionFactor)
            .velocityConversionFactor(HoodConstants.kHoodMotorVelocityConversionFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        hoodMotorConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        hoodMotorConfiguration.closedLoop
            .p(HoodConstants.kP)
            .i(0)
            .d(HoodConstants.kD);

        tryUntilOk(hoodMotor, 5, () -> hoodMotor.configure(hoodMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(hoodMotor, 5, () -> hoodEncoder.setPosition(HoodConstants.kHoodMinimumPosition.in(Radians)));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        REVUtility.sparkStickyFault = false;

        ifOkOrDefault(hoodMotor, hoodEncoder::getPosition, inputs.position);
        ifOkOrDefault(hoodMotor, hoodEncoder::getVelocity, inputs.position);
        ifOkOrDefault(hoodMotor, new DoubleSupplier[] { hoodMotor::getBusVoltage, hoodMotor::getAppliedOutput }, x -> x[0] * x[1], inputs.appliedVoltage);
        ifOkOrDefault(hoodMotor, hoodMotor::getOutputCurrent, inputs.supplyCurrentAmperes);
        ifOkOrDefault(hoodMotor, hoodMotor::getMotorTemperature, inputs.tempuratureCelcius);

        inputs.isMotorConnected = !REVUtility.sparkStickyFault;
    }

    @Override
    public void setVoltage(double voltage) {
        hoodMotor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double position, double feedforward) {
        hoodPositionController.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        hoodMotorConfiguration.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);

        tryUntilOk(hoodMotor, 5, () -> hoodMotor.configure(hoodMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // Change the neutral mode configuration in a separate thread since changing the configuration of a REV device may take a significant amount of time (~200 ms).
        brakeModeExecutor.execute(() -> {
            hoodMotor.configure(
                hoodMotorConfiguration.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast), 
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