package frc.minolib.rev;

public interface REVAbsoluteEncoder {
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
}