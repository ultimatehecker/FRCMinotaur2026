package frc.robot.subsystems.agitator;

import static frc.minolib.rev.REVUtility.tryUntilOk;
import static frc.minolib.rev.REVUtility.ifOkOrDefault;

import static edu.wpi.first.units.Units.Amps;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.minolib.rev.REVUtility;
import frc.robot.constants.AgitatorConstants;

public class AgitatorIOHardware implements AgitatorIO {
    private SparkMax leftAgitatorMotor;
    private SparkMax rightAgitatorMotor;
    
    private RelativeEncoder leftAgitatorEncoder;
    private SparkClosedLoopController leftAgitatorPIDController;

    private RelativeEncoder rightAgitatorEncoder;
    private SparkClosedLoopController rightAgitatorPIDController;

    private SparkBaseConfig agitatorMotorConfiguration;

    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    public AgitatorIOHardware() {
        leftAgitatorMotor = new SparkMax(24, MotorType.kBrushless);
        leftAgitatorEncoder = leftAgitatorMotor.getEncoder();
        leftAgitatorPIDController = leftAgitatorMotor.getClosedLoopController();

        rightAgitatorMotor = new SparkMax(26, MotorType.kBrushless);
        rightAgitatorEncoder = rightAgitatorMotor.getEncoder();
        rightAgitatorPIDController = rightAgitatorMotor.getClosedLoopController();

        agitatorMotorConfiguration = new SparkMaxConfig()
            .inverted(AgitatorConstants.kAgitatorMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) AgitatorConstants.kAgitatorMotorSupplyLimit.in(Amps), (int) AgitatorConstants.kAgitatorMotorSupplyLimit.in(Amps))
            .voltageCompensation(12.0);

        agitatorMotorConfiguration.encoder
            .positionConversionFactor(AgitatorConstants.kAgitatorMotorPositionConversionFactor)
            .velocityConversionFactor(AgitatorConstants.kAgitatorMotorVelocityConversionFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        agitatorMotorConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        agitatorMotorConfiguration.closedLoop
            .p(0)
            .i(0)
            .d(0);

        tryUntilOk(leftAgitatorMotor, 5, () -> leftAgitatorMotor.configure(agitatorMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(
            rightAgitatorMotor, 
            5, 
            () -> rightAgitatorMotor.configure(agitatorMotorConfiguration.inverted(!AgitatorConstants.kAgitatorMotorInverted), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );
    }

    @Override
    public void updateInputs(AgitatorIOInputs inputs) {
        REVUtility.sparkStickyFault = false;

        ifOkOrDefault(leftAgitatorMotor, leftAgitatorEncoder::getPosition, inputs.leftPosition);
        ifOkOrDefault(leftAgitatorMotor, leftAgitatorEncoder::getVelocity, inputs.leftVelocity);
        ifOkOrDefault(leftAgitatorMotor, new DoubleSupplier[] { leftAgitatorMotor::getBusVoltage, leftAgitatorMotor::getAppliedOutput }, x -> x[0] * x[1], inputs.leftAppliedVoltage);
        ifOkOrDefault(leftAgitatorMotor, leftAgitatorMotor::getOutputCurrent, inputs.leftSupplyCurrentAmperes);
        ifOkOrDefault(leftAgitatorMotor, leftAgitatorMotor::getMotorTemperature, inputs.leftTempuratureCelcius);
        inputs.isLeftMotorConnected = !REVUtility.sparkStickyFault;

        REVUtility.sparkStickyFault = false;

        ifOkOrDefault(rightAgitatorMotor, rightAgitatorEncoder::getPosition, inputs.rightPosition);
        ifOkOrDefault(rightAgitatorMotor, rightAgitatorEncoder::getVelocity, inputs.rightVelocity);
        ifOkOrDefault(rightAgitatorMotor, new DoubleSupplier[] { rightAgitatorMotor::getBusVoltage, rightAgitatorMotor::getAppliedOutput }, x -> x[0] * x[1], inputs.rightAppliedVoltage);
        ifOkOrDefault(rightAgitatorMotor, rightAgitatorMotor::getOutputCurrent, inputs.rightSupplyCurrentAmperes);
        ifOkOrDefault(rightAgitatorMotor, rightAgitatorMotor::getMotorTemperature, inputs.rightTempuratureCelcius);
        inputs.isLeftMotorConnected = !REVUtility.sparkStickyFault;
    }

    @Override
    public void setVoltage(double voltage) {
        leftAgitatorMotor.setVoltage(voltage);
        rightAgitatorMotor.set(voltage);
    }

    @Override
    public void setVelocity(double velocity, double feedforward) {
        leftAgitatorPIDController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
        rightAgitatorPIDController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        agitatorMotorConfiguration.closedLoop
            .p(kP)
            .i(kI)
            .d(kD);

        tryUntilOk(leftAgitatorMotor, 5, () -> leftAgitatorMotor.configure(agitatorMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
        tryUntilOk(rightAgitatorMotor, 5, () -> rightAgitatorMotor.configure(agitatorMotorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // Change the neutral mode configuration in a separate thread since changing the configuration of a REV device may take a significant amount of time (~200 ms).
        brakeModeExecutor.execute(() -> {
            leftAgitatorMotor.configure(
                agitatorMotorConfiguration.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast), 
                ResetMode.kNoResetSafeParameters, 
                PersistMode.kNoPersistParameters
            );

            rightAgitatorMotor.configure(
                agitatorMotorConfiguration.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast), 
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
