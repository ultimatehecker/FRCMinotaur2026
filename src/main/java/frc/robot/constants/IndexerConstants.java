package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class IndexerConstants {
    public static final MinoCANDevice kIndexerMotor = new MinoCANDevice(14, GlobalConstants.kRioBus);

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double simulatedKp = 0.0;
    public static final double simulatedKi = 0.0;
    public static final double simulatedKd = 0.0;
    public static final double simulatedKs = 0.0;
    public static final double simulatedKv = 0.0;
    public static final double simulatedKa = 0.0;

    public static final boolean kIndexerMotorInverted = true;
    public static final double kIndexerMotorReduction = (24.0 / 14.0);
    public static final Current kIndexerMotorSupplyLimit = Amps.of(45);
    
    public static final DCMotor kIndexerSimulatedGearbox = DCMotor.getNEO(1);

    public static final MomentOfInertia kIndexerMOI = KilogramSquareMeters.of(0.0002);

    public static final double kIndexerMotorPositionConversionFactor = (1 / kIndexerMotorReduction) * 2 * Math.PI;
    public static final double kIndexerMotorVelocityConversionFactor = 1 / kIndexerMotorPositionConversionFactor;
}
