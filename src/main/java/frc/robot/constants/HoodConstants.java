package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class HoodConstants {
    public static final AngularVelocity kHoodMaximumRotationalVelocity = RadiansPerSecond.of(1.5 * Math.PI);
    public static final AngularAcceleration kHoodMaximumRotationalAcceleration = RadiansPerSecondPerSecond.of(4 * Math.PI);

    public static final MinoCANDevice kHoodMotor = new MinoCANDevice(14, GlobalConstants.kRioBus);

    public static final Angle kHoodMinimumPosition = Degrees.of(10);
    public static final Angle kHoodMaximumPosition = Degrees.of(35); 
    public static final Angle kHoodStartingPosition = Degrees.of(10);

    public static final double kP = 4.5;
    public static final double kI = 0.0;
    public static final double kD = 0.04;
    public static final double kS = 0.15;
    public static final double kV = 0.0;
    public static final double kG = 0.0;
    public static final double kA = 0.0;

    public static final double simulatedKp = 20.0;
    public static final double simulatedKd = 0.0;
    public static final double simulatedKs = 0.0;
    public static final double simulatedKv = 0.0;
    public static final double simulatedKg = 0.0;
    public static final double simulatedKa = 0.0;

    public static final boolean kHoodMotorInverted = false;
    public static final double kHoodMotorReduction = (25.0 / 1.0) * (36.0 / 48.0) * (184.0 / 10.0);
    public static final Current kHoodMotorSupplyLimit = Amps.of(20);
    
    public static final DCMotor kHoodSimulatedGearbox = DCMotor.getNEO(1);

    public static final Distance kHoodLength = Inches.of(7);
    public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.1);

    public static final double kHoodMotorPositionConversionFactor = (1 / kHoodMotorReduction) * 2 * Math.PI;
    public static final double kHoodMotorVelocityConversionFactor = 1 / kHoodMotorPositionConversionFactor;
}
