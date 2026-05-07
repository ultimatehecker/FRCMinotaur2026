package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class ShooterConstants {
    public static final MinoCANDevice kPrimaryMotor = new MinoCANDevice(19, GlobalConstants.kRioBus);
    public static final MinoCANDevice kSecondaryMotor = new MinoCANDevice(20, GlobalConstants.kRioBus);
    public static final MinoCANDevice kThirdMotor = new MinoCANDevice(21, GlobalConstants.kRioBus);
    public static final MinoCANDevice kFourthMotor = new MinoCANDevice(21, GlobalConstants.kRioBus);

    public static final double kP = 0.0012;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.35;
    public static final double kV = 0.00262;
    public static final double kA = 0.0;

    public static final boolean kPrimaryMotorInverted = true;
    public static final double kMotorReduction = (24.0 / 18.0);
    public static final DCMotor kSimulatedGearbox = DCMotor.getKrakenX60Foc(4);

    public static final Current kMotorSupplyLimit = Amps.of(80);
    public static final Current kMotorStatorLimit = Amps.of(100);

    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.1);
    public static Transform2d kRobotTransform = new Transform2d(-0.1758535599, 0.0, new Rotation2d(0));
}
