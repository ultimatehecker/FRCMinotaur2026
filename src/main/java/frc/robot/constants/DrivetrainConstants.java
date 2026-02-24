package frc.robot.constants;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.minolib.swerve.SwerveModuleType;

public class DrivetrainConstants {
    public static final double kMaximumLinearVelocityMetersPerSecond = 4.5;
    public static final double kMaximumLinearAccelerationMetersPerSecondSquared = 9.0;
    public static final double kMaximumRotationalVelocityRadiansPerSecond = 3 * Math.PI;
    public static final double kMaximumRotationalAccelerationRadiansPerSecondSquared = 6 * Math.PI;

    public static final SwerveModuleType kSwerveModuleType = SwerveModuleType.MK4N_L2;
    
    public static final double kDriveMotorReduction = kSwerveModuleType.getDriveReduction();
    public static final DCMotor kDriveSimulatedGearbox = DCMotor.getNEO(1);
    public static final double kSteerMotorReduction = kSwerveModuleType.getSteerReduction();
    public static final DCMotor kSteerSimulatedGearbox = DCMotor.getNEO(1);

    public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
    public static final double kTrackWidth = Units.inchesToMeters(15);
    public static final double kWheelBase = Units.inchesToMeters(15);
    public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0)
    };
    
    public static final double kRobotMassKilograms = 25.00;
    public static final double kRobotMOI = 6.883;
    public static final double kWheelCOF = 1.0;
    public static final RobotConfig kPathPlannerConfiguration = new RobotConfig(
        kRobotMassKilograms,
        kRobotMOI,
        new ModuleConfig(
            kWheelRadiusMeters, 
            kMaximumLinearVelocityMetersPerSecond, 
            kWheelCOF, 
            kDriveSimulatedGearbox, 
            40, 
            1
        ),
        kModuleTranslations
    );

    public static final DriveTrainSimulationConfig kMapleSimConfiguration = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(kModuleTranslations)
        .withRobotMass(Kilogram.of(kRobotMassKilograms))
        .withGyro(COTS.ofNav2X())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                kDriveSimulatedGearbox,
                kSteerSimulatedGearbox,
                kDriveMotorReduction,
                kSteerMotorReduction,
                Volts.of(0.1),
                Volts.of(0.1),
                Meters.of(kWheelRadiusMeters),
                KilogramSquareMeters.of(0.02),
                kWheelCOF
            )
        );
}
