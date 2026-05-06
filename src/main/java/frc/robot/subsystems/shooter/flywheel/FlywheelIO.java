package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public class FlywheelIOInputs {
        public boolean isFirstShooterConnected = false;
        public double firstShooterPosition = 0.0;
        public double firstShooterVelocity = 0.0;
        public double firstShooterAppliedVoltage = 0.0;
        public double firstShooterSupplyCurrentAmperes = 0.0;
        public double firstShooterTorqueCurrentAmperes = 0.0;
        public double firstShooterTempuratureCelcius = 0.0;
        public boolean firstShooterTempuratureFault = false;

        public boolean isSecondShooterConnected = false;
        public double secondShooterSupplyCurrentAmperes = 0.0;
        public double secondShooterTempuratureCelcius = 0.0;
        public boolean secondShooterTempuratureFault = false;

        public boolean isThirdShooterConnected = false;
        public double thirdShooterSupplyCurrentAmperes = 0.0;
        public double thirdShooterTempuratureCelcius = 0.0;
        public boolean thirdShooterTempuratureFault = false;

        public boolean isFourthShooterConnected = false;
        public double fourthShooterSupplyCurrentAmperes = 0.0;
        public double fourthShooterTempuratureCelcius = 0.0;
        public boolean fourthShooterTempuratureFault = false;
    }

    public void updateInputs(FlywheelIOInputs inputs);

    public void setVoltage(double voltage);

    public default void setOL(double amperes) {}

    public void stop();

    public void setVelocity(double velocity, double feedforward);

    public void setPID(double kP, double kI, double kD, double kS, double kV, double kA);

    public default void setBrakeMode(boolean enabled) {}
}