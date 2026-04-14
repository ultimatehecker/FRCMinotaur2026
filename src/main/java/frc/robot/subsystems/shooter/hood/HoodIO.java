package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface HoodIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class HoodIOInputs {
        public boolean isMotorConnected = false;
        public double positionRadians = 0.0;
        public double velocityRadiansPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public double torqueCurrentAmperes = 0.0;
        public double supplyCurrentAmperes = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean temperatureFault = false;
    }

    public void updateInputs(HoodIOInputs inputs);

    public void setVoltage(double voltage);

    public void setOL(double amperes);

    public void stop();

    public void setPosition(double position, double feedforward);

    public void setPID(double kP, double kI, double kD);

    public default void setBrakeMode(boolean enabled) {}

    @Override
    public void refreshData();
}
