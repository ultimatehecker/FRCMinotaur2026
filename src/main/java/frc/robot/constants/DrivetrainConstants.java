package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutDimensionless;
import frc.minolib.swerve.CTRESwerveDrivetrainConstants;
import frc.minolib.swerve.SwerveModuleType;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.SimulationTunerConstants;

public class DrivetrainConstants {
    public static final SwerveModuleType kSwerveModuleType = SwerveModuleType.MK4N_L2;

    public static final LinearVelocity kMaximumLinearVelocity = MetersPerSecond.of(4.4);
    public static final LinearAcceleration kMaximumLinearAcceleration = MetersPerSecondPerSecond.of(6.0);
    public static final AngularVelocity kMaximumRotationalVelocity = RadiansPerSecond.of(3 * Math.PI);
    public static final AngularAcceleration kMaximumRotationalAcceleration = RadiansPerSecondPerSecond.of(6 * Math.PI);

    public static final double driveKp = 0.1;
    public static final double driveKi = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.124;
    public static final double driveKa = 0.0;
    public static final double driveSimulatedKp = 10.0;
    public static final double driveSimulatedKi = 0.0;
    public static final double driveSimulatedKd = 0.0;
    public static final double driveSimulatedKs = 1.5;
    public static final double driveSimulatedKv = 0.0;

    public static final boolean kDriveMotorInverted = false;
    public static final double kDriveMotorReduction = kSwerveModuleType.getDriveReduction();
    public static final Current kDriveMotorSupplyCurrentLimit = Amps.of(80);
    public static final DCMotor kDriveSimulatedGearbox = DCMotor.getKrakenX60Foc(1);

    public static final double steerKp = 100.0;
    public static final double steerKi = 0.0;
    public static final double steerKd = 0.5;
    public static final double steerKs = 0.1;
    public static final double steerKv = 1.79;
    public static final double steerKa = 0.0;
    public static final double steerSimulatedKp = 100.0;
    public static final double steerSimulatedKi = 0.0;
    public static final double steerSimulatedKd = 0.5;
    public static final double steerSimulatedKs = 0.1;
    public static final double steerSimulatedKv = 0.0;
    public static final double steerSimulatedKa = 0.0;

    public static final boolean kSteerMotorInverted = true;
    public static final double kSteerMotorReduction = kSwerveModuleType.getSteerReduction();
    public static final Current kSteerMotorStatorCurrentLimit = Amps.of(60);
    public static final DCMotor kSteerSimulatedGearbox = DCMotor.getKrakenX44Foc(1);

    public static final Distance kWheelRadius = Inches.of(1.897);
    public static final Distance kTrackWidth = Inches.of(21.75);
    public static final Distance kWheelBase = Inches.of(21.75);
    public static final Distance kDriveBaseRadius = Inches.of(Math.hypot(kTrackWidth.in(Inches) / 2.0, kWheelBase.in(Inches) / 2.0));
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidth.in(Meters) / 2.0, kWheelBase.in(Meters) / 2.0),
        new Translation2d(kTrackWidth.in(Meters) / 2.0, -kWheelBase.in(Meters) / 2.0),
        new Translation2d(-kTrackWidth.in(Meters) / 2.0, kWheelBase.in(Meters) / 2.0),
        new Translation2d(-kTrackWidth.in(Meters) / 2.0, -kWheelBase.in(Meters) / 2.0)
    };

    public static final double kDisabledDriveXStdDev = 1.0; 
    public static final double kDisabledDriveYStdDev = 1.0;
    public static final double kDisabledDriveRotStdDev = 1.0;

    public static final double kEnabledDriveXStdDev = 0.3;
    public static final double kEnabledDriveYStdDev = 0.3;
    public static final double kEnabledDriveRotStdDev = 0.3;

    public static final double kPathPlannerDriveHolonomicControllerP = 5.0;
    public static final double kPathPlannerDriveHolonomicControllerI = 0.0;
    public static final double kPathPlannerDriveHolonomicControllerD = 0.0;

    public static final double kPathPlannerSteerHolonomicControllerP = 5.0;
    public static final double kPathPlannerSteerHolonomicControllerI = 0.0;
    public static final double kPathPlannerSteerHolonomicControllerD = 0.0;
    
    public static final Mass kRobotMassKilograms = Kilograms.of(20.0);
    public static final Distance kRobotCOGHeight = Inches.of(6.0);
    public static final MomentOfInertia kRobotMOI = MomentOfInertia.ofBaseUnits(6.883, KilogramSquareMeters);
    public static final MomentOfInertia kSwerveModuleSteerMOI = MomentOfInertia.ofBaseUnits(0.02, KilogramSquareMeters);
    public static final Dimensionless kWheelCOF = Dimensionless.ofBaseUnits(1.0, Value);

    public static final ModuleConfig kPathPlannerModuleConfiguration = new ModuleConfig(
        kWheelRadius.in(Meters), 
        kMaximumLinearVelocity.in(MetersPerSecond), 
        kWheelCOF.in(Value), 
        kDriveSimulatedGearbox, 
        kDriveMotorSupplyCurrentLimit.in(Amps), 
        1
    );

    public static final RobotConfig kPathPlannerRobotConfiguration = new RobotConfig(
        kRobotMassKilograms,
        kRobotMOI,
        kPathPlannerModuleConfiguration,
        kModuleTranslations
    );

    public static final DriveTrainSimulationConfig kMapleSimConfiguration = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(kModuleTranslations)
        .withRobotMass(kRobotMassKilograms)
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                kDriveSimulatedGearbox,
                kSteerSimulatedGearbox,
                kDriveMotorReduction,
                kSteerMotorReduction,
                Volts.of(0.1),
                Volts.of(0.1),
                kWheelRadius,
                kSwerveModuleSteerMOI,
                kWheelCOF.in(Value)
            )
        );

    public static final CTRESwerveDrivetrainConstants kDrivetrain = Robot.isSimulation() ? SimulationTunerConstants.instantateConstants() : CompetitionTunerConstants.instantateConstants();
}
