package frc.minolib.rev;

import com.revrobotics.spark.ClosedLoopSlot;

import frc.minolib.phoenix.MechanismRatio;

public interface REVMotorController {
    /** Returns the CAN ID of the device. */
    public int getDeviceID();

    /** Updates inputs for logging. Should be called periodically. */
    public void updateInputs();
  
    /** Configures the motor. Should be called on construction or when recovering from power loss or fault. Non-blocking compared to CTRE. */
    public boolean setConfiguration();
  
    /** Sets percent output from -1.0 to 1.0. */
    public void setPercentOutput(double percent);
  
    /** Sets voltage output. */
    public void setVoltageOutput(double voltage);
  
    /** Closed-loop position mode. Position setpoint defined in MechanismRatio units. */
    public void setPositionSetpoint(ClosedLoopSlot slot, double setpoint);
  
    /** Closed-loop position mode with feed-forward. Position setpoint defined in MechanismRatio units. */
    public void setPositionSetpoint(ClosedLoopSlot slot, double setpoint, double feedforwardVolts);
  
    /** Closed-loop velocity mode. Velocity setpoint defined in MechanismRatio units. */
    public void setVelocitySetpoint(ClosedLoopSlot slot, double setpoint);
  
    /** Closed-loop velocity mode with feed-forward. Velocity setpoint defined in MechanismRatio units. */
    public void setVelocitySetpoint(ClosedLoopSlot slot, double setpoint, double feedforwardVolts);
  
    /** Returns the percent output (-1.0 to 1.0) as reported by the device. */
    public double getPercentOutput();
  
    /** Returns the percent output (-1.0 to 1.0) as reported by the device. Sign corresponds to the physical direction of the motor. Useful for simulation. */
    public double getPhysicalPercentOutput();
  
    /** Returns whether the motor is inverted. */
    public boolean getInverted();
  
    /** Sets the sensor zero-point to the current position. */
    public void zeroSensorPosition();
  
    /** Sets the sensor position to the given value. Uses MechanismRatio units. */
    public void setSensorPosition(double pos);
  
    /** Returns the sensor position. Uses MechanismRatio units. */
    public double getSensorPosition();
  
    /** Returns the sensor velocity. Uses MechanismRatio units. */
    public double getSensorVelocity();
  
    /** Returns the MechaismRatio. */
    public MechanismRatio getMechanismRatio();
  
    /** Convert MechanismRatio position to native sensor position. */
    public double toNativeSensorPosition(double position);
  
    /** Convert from native sensor position. Returns position in MechanismRatio units. */
    public double fromNativeSensorPosition(double position);
  
    /** Convert MechanismRatio velocity to native sensor velocity. */
    public double toNativeSensorVelocity(double velocity);
  
    /** Convert from native sensor velocity. Returns velocity in MechanismRatio units. */
    public double fromNativeSensorVelocity(double velocity);
  
    /** Sets the simulated angular position and velocity of the sensor in mechanism units. */
    public void setSimulatedSensorPositionAndVelocity(double position, double velocity, double dt, MechanismRatio mr);
  
    /** Sets the simulated angular velocity of the sensor in mechanism units. Also sets the simulated position. */
    public void setSimulatedSensorVelocity(double velocity, double dt, MechanismRatio mr);
}
