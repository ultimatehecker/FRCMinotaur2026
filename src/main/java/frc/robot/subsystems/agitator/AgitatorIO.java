package frc.robot.subsystems.agitator;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface AgitatorIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class AgitatorIOInputs {
        public boolean isLeftMotorConnected = true;
        public double leftPosition = 0.0;
        public double leftVelocity = 0.0;
        public double leftAcceleration = 0.0;
        public double leftAppliedVoltage = 0.0;
        public double leftSupplyCurrentAmperes = 0.0;
        public double leftTempuratureCelcius = 0.0;

        public boolean isRightMotorConnected = true;
        public double rightPosition = 0.0;
        public double rightVelocity = 0.0;
        public double rightAcceleration = 0.0;
        public double rightAppliedVoltage = 0.0;
        public double rightSupplyCurrentAmperes = 0.0;
        public double rightTempuratureCelcius = 0.0;
    }

    public void updateInputs(AgitatorIOInputs inputs);

    public void setVoltage(double voltage);

    public void setVelocity(double velocity, double feedforward);

    public void setPID(double kP, double kI, double kD);

    public default void setBrakeMode(boolean enabled) {}

    public void stop();

    @Override
    public void refreshData();
}
