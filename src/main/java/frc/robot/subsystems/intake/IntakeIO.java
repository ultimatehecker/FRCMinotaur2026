package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface IntakeIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class IntakeIOInputs {
        public boolean rollerMotorConnected = true;
        public double rollerPosition = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerAcceleration = 0.0;
        public double rollerAppliedVoltage = 0.0;
        public double rollerSupplyCurrentAmperes = 0.0;
        public double rollerTorqueCurrentAmperes = 0.0;
        public double rollerTemperatureCelsius = 0.0;

        public boolean pivotMotorConnected = true;
        public double pivotPosition = 0.0;
        public double pivotVelocity = 0.0;
        public double pivotAcceleration = 0.0;
        public double pivotAppliedVoltage = 0.0;
        public double pivotSupplyCurrentAmperes = 0.0;
        public double pivotMotorTempuratureCelcius = 0.0;
    }

    public enum IntakeControlMode {
        BRAKE,
        COAST,
        VOLTAGE_CONTROL,
        VELOCITY_CONTROLLED
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setPivotVoltage(double voltage) {}

    public default void setRollerVoltage(double voltage) {}

    public default void setPivotPosition(double position, double feedforward) {}

    public default void setRollerTorqueCurrent(double amperes) {}

    public default void stopRollers() {}

    public default void setPivotPID(double kP, double kI, double kD) {}

    public default void setBrakeMode(boolean enabled) {}

    @Override
    public void refreshData(); 
}
