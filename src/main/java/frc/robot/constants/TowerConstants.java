package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import org.opencv.core.Mat;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class TowerConstants {
    public static final MinoCANDevice kTopMotor = new MinoCANDevice(20, GlobalConstants.kRioBus);
    public static final MinoCANDevice kBottomMotor = new MinoCANDevice(19, GlobalConstants.kRioBus);

    public static final double topkP = 0.0;
    public static final double topkD = 0.0;
    public static final double topkS = 0.0;
    public static final double topkV = 0.0;
    public static final double topkA = 0.0;

    public static final boolean kTopMotorInverted = true;
    public static final double kTopMotorReduction = (3.0 / 1.0);
    public static final DCMotor kTopSimulatedGearbox = DCMotor.getKrakenX60Foc(1);

    public static final Current kTopRollerStatorLimit = Amps.of(70);
    public static final Current kTopRollerSupplyLimit = Amps.of(30);

    public static final double bottomkP = 0.0;
    public static final double bottomkD = 0.0;
    public static final double bottomkS = 0.0;
    public static final double bottomkV = 0.0;
    public static final double bottomkA = 0.0;

    public static final boolean kBottomRollerMotorInverted = false;
    public static final double kBottomMotorReduction = (3.0 / 1.0);
    public static final DCMotor kBottomRollerSimulatedGearbox = DCMotor.getFalcon500Foc(1);

    public static final Current kBottomRollerStatorLimit = Amps.of(70);
    public static final Current kBottomRollerSupplyLimit = Amps.of(30);

    public static final MomentOfInertia kTopRollerMOI = KilogramSquareMeters.of(0.0002);
    public static final MomentOfInertia kBottomRollerMOI = KilogramSquareMeters.of(0.0002);
}
