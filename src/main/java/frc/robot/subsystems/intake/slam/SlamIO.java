package frc.robot.subsystems.intake.slam;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
    @AutoLog
    public class SlamIOInputs {
        public boolean isMotorConnected = false;
        public double positionRadians = 0.0;
        public double velocityRadiansPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmperes = 0.0;
        public double torqueCurrentAmperes = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean temperatureFault = false;
    }

    public void updateInputs(SlamIOInputs inputs);

    public void setVoltage(double voltage);

    public void setOL(double amperes);

    public void stop();

    public void resetPosition();

    public void setPosition(double positionRadians);

    public default void setPID(double kP, double kI, double kD, double kS, double kV, double kG, double kA) {}

    public default void setBrakeMode(boolean enabled) {}
}
