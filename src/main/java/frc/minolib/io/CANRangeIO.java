package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface CANRangeIO {
    @AutoLog
    public static class CANRangeInputs {
        public boolean isSensorConnected = false;
        public StatusCode status = StatusCode.OK;
        public double distanceMeters = 0.0;          
        public boolean hasDetection = false;    
        public double busVoltage = 0.0;           
        public double temperatureCelcius = 0.0;   
    }

    /** Returns the CAN ID of the CANrange device. */
    public int getDeviceID();

    /** Configures the CANrange sensor. Should be called during init or after resets. Returns true if configuration succeeds. */
    public boolean setConfiguration();

    /** Updates readings into the supplied inputs object. */
    public void updateInputs(CANRangeInputs inputs);

    /** Returns the measured distance in meters. */
    public double getDistanceMeters();

    /** Returns whether the proximity threshold is currently triggered. */
    public boolean isProximityDetected();

    /** Returns the current CAN bus voltage at the sensor. */
    public double getBusVoltage();

    /** Returns sensor/device temperature in Celsius, if provided. */
    public double getTemperatureCelcius();

    /** Simulation helper: set distance and proximity state over simulated time (seconds). */
    public void setSimulatedDistanceAndProximity(double distance, boolean proximity, double dt);
}
