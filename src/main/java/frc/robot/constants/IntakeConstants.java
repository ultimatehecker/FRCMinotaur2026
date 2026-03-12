package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class IntakeConstants {
    public static final AngularVelocity kRollerMaximumRotationalVelocity = RadiansPerSecond.of(4.2);
    public static final AngularAcceleration kRollerMaximumRotationalAcceleration = RadiansPerSecondPerSecond.of(6.0);
    public static final AngularVelocity kPivotMaximumRotationalVelocity = RadiansPerSecond.of(3 * Math.PI);
    public static final AngularAcceleration kPivotMaximumRotationalAcceleration = RadiansPerSecondPerSecond.of(6 * Math.PI);

    public static final MinoCANDevice kPivotMotor = new MinoCANDevice(14, GlobalConstants.kRioBus);
    public static final MinoCANDevice kRollerMotor = new MinoCANDevice(15, GlobalConstants.kCANivoreBus);
    public static final MinoCANDevice kPivotAbsoluteEncoder = new MinoCANDevice(16, GlobalConstants.kRioBus);

    public static final AngularVelocity kRollerIdleThreshold = RadiansPerSecond.of(0.5);

    public static final Angle kIntakeMinimumPosition = Degrees.of(45);
    public static final Angle kIntakeMaximumPosition = Degrees.of(170);
    public static final Angle kIntakeStartingPosition = Degrees.of(45);

    public static final Angle kPivotAbsoluteEncoderOffset = Rotations.of(0);

    public static final Mass kIntakeMass = Kilograms.of(20.0);
    public static final Distance kIntakeLength = Inches.of(12.0);
    public static final MomentOfInertia kRollerMOI = MomentOfInertia.ofBaseUnits(0.02, KilogramSquareMeters);
    public static final MomentOfInertia kPivotMOI = MomentOfInertia.ofBaseUnits(0.02, KilogramSquareMeters);

    public static final double pivotKp = 0.01;
    public static final double pivotKd = 0.0;
    public static final double pivotKs = 0.0;
    public static final double pivotKv = 0.0;
    public static final double pivotKCos = 0.0;
    public static final double pivotKa = 0.0;
    public static final double pivotSimulatedKp = 10.0;
    public static final double pivotSimulatedKd = 0.01;
    public static final double pivotSimulatedKs = 0.0;
    public static final double pivotSimulatedKv = 0.0;
    public static final double pivotSimulatedKCos = 0.0;
    public static final double pivotSimulatedKa = 0.0;

    public static final boolean kPivotMotorInverted = false;
    public static final double kPivotMotorReduction = (25.0 / 1) * (32.0 / 16.0);
    public static final Current kPivotMotorSupplyLimit = Amps.of(30);
    public static final DCMotor kPivotSimulatedGearbox = DCMotor.getNEO(1);

    public static final double kPivotMotorPositionConversionFactor = (1 / kPivotMotorReduction) * 2 * Math.PI;
    public static final double kPivotMotorVelocityConversionFactor = 1 / kPivotMotorPositionConversionFactor;

    public static final double rollerKp = 0.0;
    public static final double rollerKd = 0.0;
    public static final double rollerKs = 0.0;
    public static final double rollerKv = 0.0;
    public static final double rollerKa = 0.0;
    public static final double rollerSimulatedKp = 0.0;
    public static final double rollerSimulatedKd = 0.0;
    public static final double rollerSimulatedKs = 0.0;
    public static final double rollerSimulatedKv = 0.0;
    public static final double rollerSimulatedKa = 0.0;

    public static final boolean kRollerMotorInverted = false;
    public static final double kRollerMotorReduction = 1;
    public static final Current kRollerMotorSupplyLimit = Amps.of(50);
    public static final DCMotor kRollerSimulatedGearbox = DCMotor.getFalcon500Foc(1);

    public static final double kRollerVelocityFilterTimeConstant = 0.1;
}
