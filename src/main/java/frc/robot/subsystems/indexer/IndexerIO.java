package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface IndexerIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class IndexerIOInputs {
        public boolean isMotorConnected = true;
        public double position = 0.0;
        public double velocity = 0.0;
        public double acceleration = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmperes = 0.0;
        public double tempuratureCelcius = 0.0;
    }

    public enum IndexerControlMode {
        BRAKE,
        COAST,
        VOLTAGE_CONTROL,
        VELOCITY_CONTROLLED
    }

    public void updateInputs(IndexerIOInputs inputs);

    public void setVoltage(double voltage);

    public void setVelocity(double velocity, double feedforward);

    public void setPID(double kP, double kI, double kD);

    public default void setBrakeMode(boolean enabled) {}

    public void stop();

    @Override
    public void refreshData();
}
