package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.minolib.hardware.CANDeviceID;

public class IndexerIOHardware implements IndexerIO {
    private SparkMax indexerMotor;
    
    private RelativeEncoder indexerEncoder;
    private SparkClosedLoopController indexerPIDController;

    private SparkBaseConfig configuration;

    public IndexerIOHardware(CANDeviceID indexerDevice) {
        indexerMotor = new SparkMax(indexerDevice.deviceNumber, MotorType.kBrushless);
        indexerEncoder = indexerMotor.getEncoder();
        indexerPIDController = indexerMotor.getClosedLoopController();

        configuration = new SparkMaxConfig()
        //    .inverted(IntakeConstants.kPivotMotorInverted)
            .idleMode(IdleMode.kBrake)
         //   .smartCurrentLimit((int) IntakeConstants.kPivotMotorSupplyLimit.in(Amps), 50)
            .voltageCompensation(12.0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {

    }

    @Override
    public void setVoltage(double voltage) {

    }

    @Override
    public void setVelocity(double velocity, double feedforward) {

    }

    @Override
    public void setPID(double kP, double kI, double kD) {

    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void setBrakeMode(boolean enabled) {

    }

    @Override
    public void refreshData() {

    }
}
