package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

import frc.minolib.hardware.MinoCANDevice;

public class ElevatorConstants {
    public static final AngularVelocity kMaximumLinearVelocity = RadiansPerSecond.of(30);
    public static final AngularAcceleration kMaximumLinearAcceleration = RadiansPerSecondPerSecond.of(60.0);

    public static final MinoCANDevice kMotor = new MinoCANDevice(26, GlobalConstants.kRioBus);

    public static final Angle kRetractedPosition = Radians.of(0.0);
    public static final Angle kDeployedPosition = Radians.of(18.0);
    public static final Angle kClimbedPosition = Radians.of(6.0);

    public static final double kP = 0.1;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kA = 0.0;

    public static final boolean kMotorInverted = false;
    public static final double kMotorReduction = (25.0 / 1);
    public static final DCMotor kSimulatedGearbox = DCMotor.getKrakenX60Foc(1);

    public static final Current kMotorStatorLimit = Amps.of(80);
    public static final Current kMotorSupplyLimit = Amps.of(40);

    public static final Mass kMass = Kilograms.of(5.89);
    public static final Distance Length = Inches.of(8.0);
    public static final MomentOfInertia kMOI = MomentOfInertia.ofBaseUnits(0.05, KilogramSquareMeters);
}
