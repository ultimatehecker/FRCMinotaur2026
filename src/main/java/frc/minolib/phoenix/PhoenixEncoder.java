package frc.minolib.phoenix;

import com.ctre.phoenix6.StatusCode;

public interface PhoenixEncoder {
    /** Performs a non-blocking update on the inputs. */
    public StatusCode updateInputs();

    /** Performs a blocking update on the inputs. */
    public StatusCode waitForInputs(final double timeoutSeconds);

    /** Sets the sensor zero-point to the current position. */
    public void zero();

    /** Sets the sensor position to the given value. Uses MechanismRatio units. */
    public void setPosition(double pos);

    /** Returns the sensor position. Uses MechanismRatio units. */
    public double getPosition();

    /** Returns the sensor absolute position. Uses MechanismRatio units. */
    public double getAbsolutePosition();

    /** Returns the sensor velocity. Uses MechanismRatio units. */
    public double getVelocity();

    /** Sets the simulated angular velocity of the sensor in mechanism units. Also sets the simulated position. */
    public void setSimSensorVelocity(double vel, double dt);
}