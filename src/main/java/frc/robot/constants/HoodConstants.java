package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class HoodConstants {
    public static final AngularVelocity kMaximumRotationalVelocity = RotationsPerSecond.of(16.84 / 60);
    public static final AngularAcceleration kMaximumRotationalAcceleration = RotationsPerSecondPerSecond.of(100 / 60);
    public static final double kMaximumRotationalJerk = 200 / 60;

    public static final MinoCANDevice kMotor = new MinoCANDevice(25, GlobalConstants.kRioBus);

    public static final Angle kZeoredPosition = Degrees.of(11.5);
    public static final Angle kMinimumPosition = Degrees.of(12.25);
    public static final Angle kMaximumPosition = Degrees.of(47); 

    public static final double kP = 550.0;
    public static final double kI = 0.0;
    public static final double kD = 3.0;
    public static final double kS = 1.8;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final boolean kMotorInverted = false;
    public static final double kMotorReduction = (25.0 / 1.0) * (36.0 / 48.0) * (190.0 / 10.0);
    public static final DCMotor kSimulatedGearbox = DCMotor.getKrakenX44Foc(1);

    public static final Current kMotorSupplyLimit = Amps.of(30);
    public static final Current kMotorStatorLimit = Amps.of(70);

    public static final Distance kLength = Inches.of(7);
    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.1);
}
