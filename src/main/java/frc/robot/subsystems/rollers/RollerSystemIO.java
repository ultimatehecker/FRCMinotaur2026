package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
    @AutoLog
    public class RollerSystemIOInputs {
        public boolean isMotorConnected = false;
        public double positionRadians = 0.0;
        public double velocityRadiansPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public double torqueCurrentAmperes = 0.0;
        public double supplyCurrentAmperes = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean temperatureFault = false;
    }

    public void updateInputs(RollerSystemIOInputs inputs);

    public void setVoltage(double voltage);

    public void setOL(double amperes);

    public void stop();

    public default void setCurrentLimit(double currentLimit) {}

    public default void setBrakeMode(boolean enabled) {}
}