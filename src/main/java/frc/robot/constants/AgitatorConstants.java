package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public class AgitatorConstants {
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

    public static final boolean kAgitatorMotorInverted = false;
    public static final double kAgitatorMotorReduction = (1.0 / 1.0);
    public static final Current kAgitatorMotorSupplyLimit = Amps.of(50);
    
    public static final DCMotor kAgitatorSimulatedGearbox = DCMotor.getNEO(1);

    public static final MomentOfInertia kAgitatorMOI = KilogramSquareMeters.of(0.0002);

    public static final double kAgitatorMotorPositionConversionFactor = (1 / kAgitatorMotorReduction) * 2 * Math.PI;
    public static final double kAgitatorMotorVelocityConversionFactor = 1 / kAgitatorMotorPositionConversionFactor;
}
