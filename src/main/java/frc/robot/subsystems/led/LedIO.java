package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.wpilibj.util.Color;

public interface LedIO {
    public default void setLEDs(Color color) {}

    public default void setLEDs(int red, int green, int blue) {}

    public default void setControl(ControlRequest controlRequest) {}

    public default void clearAnimation() {}
}
