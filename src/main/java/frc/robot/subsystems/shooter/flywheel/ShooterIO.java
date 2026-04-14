package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface ShooterIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class ShooterIOInputs {
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

    public void updateInputs(ShooterIOInputs inputs);

    public void setVoltage(double voltage);

    public default void setOL(double amperes) {}

    public void stop();

    public void setVelocity(double velocity, double feedforward);

    public void setPID(double kP, double kI, double kD);

    public default void setBrakeMode(boolean enabled) {}

    @Override
    public void refreshData();
}