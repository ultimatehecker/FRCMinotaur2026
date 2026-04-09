package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class ShooterConstants {
    public static final MinoCANDevice kPrimaryShooterMotor = new MinoCANDevice(19, GlobalConstants.kRioBus);
    public static final MinoCANDevice kSecondaryShooterMotor = new MinoCANDevice(20, GlobalConstants.kRioBus);
    public static final MinoCANDevice kThirdShooterMotor = new MinoCANDevice(21, GlobalConstants.kRioBus);

    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final boolean kPrimaryShooterMotorInverted = true;
    public static final boolean kOtherShooterMotorInverted = true;
    public static final double kShooterMotorReduction = (24.0 / 18.0);
    public static final DCMotor kShooterSimulatedGearbox = DCMotor.getKrakenX60Foc(4);

    public static final Current kShooterMotorSupplyLimit = Amps.of(70);
    public static final Current kShooterMotorStatorLimit = Amps.of(80);

    public static final MomentOfInertia kShooterMOI = KilogramSquareMeters.of(0.1);

    public static Transform2d kRobotToShooterTransform = new Transform2d(-0.1758535599, 0.0, new Rotation2d(Math.PI));
}
