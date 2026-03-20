package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class HangConstants {
    public static final double pivotKp = 0.0;
    public static final double pivotKd = 0.0;
    public static final double pivotKs = 0.0;
    public static final double pivotKv = 0.0;
    public static final double pivotKa = 0.0;

    public static final double simulatedPivotKp = 0.0;
    public static final double simulatedPivotKd = 0.0;
    public static final double simulatedPivotKs = 0.0;
    public static final double simulatedPivotKv = 0.0;
    public static final double simulatedPivotKa = 0.0;

    public static final boolean kPivotMotorInverted = true;
    public static final double kPivotMotorReduction = (1.0 / 1.0) * 2 * Math.PI;
    public static final Current kPivotMotorSupplyLimit = Amps.of(80);
    public static final DCMotor kPivotSimulatedGearbox = DCMotor.getNEO(1);

    public static final double winderkP = 0.0;
    public static final double winderkI = 0.0;
    public static final double winderkD = 0.0;
    public static final double winderkS = 0.0;
    public static final double winderkV = 0.0;
    public static final double winderkA = 0.0;

    public static final double simulatedWinderKp = 0.0;
    public static final double simulatedWinderKd = 0.0;
    public static final double simulatedWinderKs = 0.0;
    public static final double simulatedWinderKv = 0.0;
    public static final double simulatedWinderKa = 0.0;

    public static final boolean kWinderMotorInverted = false;
    public static final double kWinderRollerMotorReduction = (1.0 / 1.0) * 2 * Math.PI;
    public static final Current kWinderMotorSupplyLimit = Amps.of(50);
    public static final DCMotor kWinderSimulatedGearbox = DCMotor.getNEO(1);

    public static final MomentOfInertia kPivotMOI = KilogramSquareMeters.of(0.0002);
    public static final MomentOfInertia kWinderMOI = KilogramSquareMeters.of(0.0002);

    public static final double kPivotMotorPositionConversionFactor = (1 / kPivotMotorReduction) * 2 * Math.PI;
    public static final double kPivotMotorVelocityConversionFactor = 1 / kPivotMotorPositionConversionFactor;
    public static final double kWinderMotorPositionConversionFactor = (1 / kWinderRollerMotorReduction) * 2 * Math.PI;
    public static final double kWinderMotorVelocityConversionFactor = 1 / kWinderMotorPositionConversionFactor;
}
