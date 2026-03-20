package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface ShooterIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class ShooterIOInputs {
        public boolean isFirstShooterConnected = true;
        public double firstShooterPosition = 0.0;
        public double firstShooterVelocity = 0.0;
        public double firstShooterAppliedVoltage = 0.0;
        public double firstShooterSupplyCurrentAmperes = 0.0;
        public double firstShooterTorqueCurrentAmperes = 0.0;
        public double firstShooterTempuratureCelcius = 0.0;

        public boolean isSecondShooterConnected = true;
        public double secondShooterSupplyCurrentAmperes = 0.0;
        public double secondShooterTorqueCurrentAmperes = 0.0;
        public double secondShooterTempuratureCelcius = 0.0;

        public boolean isThirdShooterConnected = true;
        public double thirdShooterSupplyCurrentAmperes = 0.0;
        public double thirdShooterTorqueCurrentAmperes = 0.0;
        public double thirdShooterTempuratureCelcius = 0.0;
    }

    public void updateInputs(ShooterIOInputs inputs);

    public void setVoltage(double voltage);

    public void setVelocity(double velocity, double feedforward);

    public void setPID(double kP, double kI, double kD);

    public default void setBrakeMode(boolean enabled) {}

    public void stop();

    @Override
    public void refreshData();
}