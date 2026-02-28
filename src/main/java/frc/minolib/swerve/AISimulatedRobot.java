package frc.minolib.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DrivetrainConstants;

public class AISimulatedRobot {
    public static final Pose2d[] kRobotQueeningPositions = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d()),
        new Pose2d(-1, 0, new Rotation2d())
    };

    public static final Pose2d[] kRobotStartingPositions = new Pose2d[] {
        new Pose2d(15, 6, new Rotation2d()),
        new Pose2d(15, 4, new Rotation2d()),
        new Pose2d(15, 2, new Rotation2d()),
        new Pose2d(2, 6, new Rotation2d()),
        new Pose2d(2, 4, new Rotation2d()),
        new Pose2d(2, 2, new Rotation2d())
    };

    private final SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;

    public AISimulatedRobot(int id) {
        this.id = id;
        this.queeningPose = kRobotQueeningPositions[id];
        this.driveSimulation = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(DrivetrainConstants.kMapleSimConfiguration, queeningPose));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    }

    private static final RobotConfig kAIPathPlannerRobotConfiguration = new RobotConfig(
        55, 
        8, 
        new ModuleConfig(Units.inchesToMeters(2), 4.5, 1.2, DCMotor.getNEO(1).withReduction(6), 60, 1), 
        Meters.convertFrom(27.5, Inches)
    );
}
