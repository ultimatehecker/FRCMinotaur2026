package frc.robot.subsystems.tower;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface TowerIO {
    @AutoLog
    public class TowerIOInputs {
        public boolean isTopRollerMotorConnected = true;
        public double topRollerPosition = 0.0;
        public double topRollerVelocity = 0.0;
        public double topRollerAcceleration = 0.0;
        public double topRollerAppliedVoltage = 0.0;
        public double topRollerSupplyCurrentAmperes = 0.0;
        public double topRollerTempuratureCelcius = 0.0;

        public boolean isBottomRollerMotorConnected = true;
        public double bottomRollerPosition = 0.0;
        public double bottomRollerVelocity = 0.0;
        public double bottomRollerAcceleration = 0.0;
        public double bottomRollerAppliedVoltage = 0.0;
        public double bottomRollerSupplyCurrentAmperes = 0.0;
        public double bottomRollerTempuratureCelcius = 0.0;
    }

    public void updateInputs(TowerIOInputs inputs);

    public void setTopRollerVoltage(double voltage);

    public void setBottomRollerVoltage(double voltage);

    public void setTopRollerVelocity(double velocity, double feedforward);

    public void setBottomRollerVelocity(double velocity, double feedforward);

    public void setTopRollerPID(double kP, double kI, double kD);

    public void setBottomRollerPID(double kP, double kI, double kD);

    public default void setBrakeMode(boolean enabled) {}

    public void stopRollers();
}
