package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface HangIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class HangIOInputs {
        public boolean isPivotMotorConnected = true;
        public double pivotPosition = 0.0;
        public double pivotVelocity = 0.0;
        public double pivotAcceleration = 0.0;
        public double pivotAppliedVoltage = 0.0;
        public double pivotSupplyCurrentAmperes = 0.0;
        public double pivotTempuratureCelcius = 0.0;

        public boolean isWinderMotorConnected = true;
        public double winderPosition = 0.0;
        public double winderVelocity = 0.0;
        public double winderAcceleration = 0.0;
        public double winderAppliedVoltage = 0.0;
        public double winderSupplyCurrentAmperes = 0.0;
        public double winderTempuratureCelcius = 0.0;
    }

    public default void updateInputs(HangIOInputs inputs) {}

    public default void setPivotVoltage(double voltage) {}

    public default void setWinderVoltage(double voltage) {}

    public default void setPivotPosition(double position, double feedforward) {}

    public default void setWinderPosition(double position, double feedforward) {}

    public default void setPivotPID(double kP, double kI, double kD) {}

    public default void setWinderPID(double kP, double kI, double kD) {}

    public default void setBrakeMode(boolean enabled) {}

    public default void stop() {}

    @Override
    public void refreshData(); 
}
