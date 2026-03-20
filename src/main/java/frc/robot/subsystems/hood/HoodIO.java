package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface HoodIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class HoodIOInputs {
        public boolean isMotorConnected = true;
        public double position = 0.0;
        public double velocity = 0.0;
        public double acceleration = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmperes = 0.0;
        public double tempuratureCelcius = 0.0;
    }

    public void updateInputs(HoodIOInputs inputs);

    public void setVoltage(double voltage);

    public void setPosition(double position, double feedforward);

    public void setPID(double kP, double kI, double kD);

    public default void setBrakeMode(boolean enabled) {}

    public void stop();

    @Override
    public void refreshData();
}
