package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import org.opencv.core.Mat;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class TowerConstants {
    public static final MinoCANDevice kTopTowerMotor = new MinoCANDevice(14, GlobalConstants.kRioBus);
    public static final MinoCANDevice kBottomTowerMotor = new MinoCANDevice(14, GlobalConstants.kRioBus);

    public static final double topRollerkP = 0.0;
    public static final double topRollerkD = 0.0;
    public static final double topRollerkS = 0.0;
    public static final double topRollerkV = 0.0;
    public static final double topRollerkA = 0.0;

    public static final double simulatedTopRollerkP = 0.0;
    public static final double simulatedTopRollerkD = 0.0;
    public static final double simulatedTopRollerkS = 0.0;
    public static final double simulatedTopRollerkV = 0.0;
    public static final double simulatedTopRollerkA = 0.0;

    public static final boolean kTopRollerMotorInverted = true;
    public static final double kTopRollerMotorReduction = (1.0 / 1.0) * 2 * Math.PI;
    public static final Current kTopRollerMotorSupplyLimit = Amps.of(20);
    public static final DCMotor kTopRollerSimulatedGearbox = DCMotor.getNEO(1);

    public static final double bottomRollerkP = 0.0;
    public static final double bottomRollerkI = 0.0;
    public static final double bottomRollerkD = 0.0;
    public static final double bottomRollerkS = 0.0;
    public static final double bottomRollerkV = 0.0;
    public static final double bottomRollerkA = 0.0;

    public static final double simulatedBottomRollerkP = 0.0;
    public static final double simulatedBottomRollerkD = 0.0;
    public static final double simulatedBottomRollerkS = 0.0;
    public static final double simulatedBottomRollerkV = 0.0;
    public static final double simulatedBottomRollerkA = 0.0;

    public static final boolean kBottomRollerMotorInverted = false;
    public static final double kBottomRollerMotorReduction = (1.0 / 1.0) * 2 * Math.PI;
    public static final Current kBottomRollerMotorSupplyLimit = Amps.of(20);
    public static final DCMotor kBottomRollerSimulatedGearbox = DCMotor.getNEO(1);

    public static final MomentOfInertia kTopRollerMOI = KilogramSquareMeters.of(0.0002);
    public static final MomentOfInertia kBottomRollerMOI = KilogramSquareMeters.of(0.0002);

    public static final double kTopRollerMotorPositionConversionFactor = (1 / kTopRollerMotorReduction) * 2 * Math.PI;
    public static final double kTopRollerMotorVelocityConversionFactor = 1 / kTopRollerMotorPositionConversionFactor;
    public static final double kBottomRollerMotorPositionConversionFactor = (1 / kBottomRollerMotorReduction) * 2 * Math.PI;
    public static final double kBottomRollerMotorVelocityConversionFactor = 1 / kBottomRollerMotorPositionConversionFactor;
}
