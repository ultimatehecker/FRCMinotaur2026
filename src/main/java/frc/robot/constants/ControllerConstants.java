package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class ControllerConstants {
    public static final SimControllerType kSimControllerType = SimControllerType.XBOX;
    public static final Time kPOVDebounceTimeSeconds = Seconds.of(0.1);

    public enum SimControllerType {
        XBOX,
        DUAL_SENSE,
        VEX
    }

    public static final int kDriverControllerPort = 0;
}