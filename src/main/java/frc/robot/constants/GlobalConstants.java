package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

public class GlobalConstants {
    public static double kLoopBackTimeSeconds = 1.0;
    public static final double kLowBatteryVoltage = 10.0;
    public static final double kLowBatteryDisabledTime = 1.5;
    public static final double kCANErrorTimeThreshold = 0.5; // Seconds to disable alert
    public static final double kCANivoreTimeThreshold = 0.5;

    public static final Mode kSimulationMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimulationMode;
    public static final boolean kTuningMode = true;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
