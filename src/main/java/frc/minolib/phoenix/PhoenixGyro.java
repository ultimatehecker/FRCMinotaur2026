package frc.minolib.phoenix;

import com.ctre.phoenix6.StatusCode;

public interface PhoenixGyro {
    /** Performs a non-blocking update on the inputs. */
    public StatusCode updateInputs();

    /** Performs a blocking update on the inputs. */
    public StatusCode waitForInputs(final double timeoutSeconds);
    
    /** Sets the continuous yaw zero-point to the current position. */
    public void zeroContinuousYaw();
  
    /** Sets the continuous yaw to the given value in radians. */
    public void setContinuousYaw(double rad);
  
    /** Returns the roll (rotation around +X axis) in radians. Roll to the right is positive. */
    public double getRoll();
  
    /** Returns the pitch (rotation around +Y axis) in radians. Pitch down is positive. */
    public double getPitch();
  
    /** Returns the continuous yaw in radians. CCW is positive. */
    public double getContinuousYaw();
  
    /** Returns the roll rate (rotation around +X axis) in radians per sec. Roll to the right is positive. */
    public double getRollRate();
  
    /** Returns the pitch rate (rotation around +Y axis) in radians per sec. Pitch down is positive. */
    public double getPitchRate();
  
    /** Returns the yaw rate in radians per sec. CCW is positive. */
    public double getYawRate();
  
    /** Sets the simulated continuous yaw in radians. Note that updates must be continuous. */
    public void setSimContinuousYaw(double rad);
}
