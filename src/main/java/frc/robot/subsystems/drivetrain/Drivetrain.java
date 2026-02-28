package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.swerve.pathplanner.PathPlannerLogging;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.minolib.wpilib.RobotTime;
import frc.robot.RobotState;
import frc.robot.constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
    private DrivetrainIO io;
    private RobotState robotState;

    private DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged(); // TODO: Log modules in io to make this cleaner maybe?
    private ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    private ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    private ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
    private ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

    private final Object moduleIOLock = new Object();

    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final ApplyRobotSpeeds pathplannerRequest = new ApplyRobotSpeeds()
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withDesaturateWheelSpeeds(true);

    public Drivetrain(DrivetrainIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (moduleIOLock) {
                io.updateModuleInputs(frontLeftInputs, frontRightInputs, backLeftInputs, backRightInputs);
            }
        }, io);

        configurePathPlanner();
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.updateDrivetrainInputs(inputs);

        synchronized (moduleIOLock) {
            Logger.processInputs("Drivetrain", inputs);
            Logger.processInputs("Drivetrain/Front Left", frontLeftInputs);
            Logger.processInputs("Drivetrain/Front Right", frontRightInputs);
            Logger.processInputs("Drivetrain/Back Left", backLeftInputs);
            Logger.processInputs("Drivetrain/Back Right", backRightInputs);
        }

        if (DriverStation.isDisabled()) {
            configureStandardDevsForDisabled();
        } else {
            configureStandardDevsForEnabled();
        }

        Logger.recordOutput("Drivetrain/LatencyPeriodicSeconds", RobotTime.getTimestampSeconds() - timestamp);
        Logger.recordOutput("Drivetrain/CurrentCommand", (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    }

    private void configurePathPlanner() {
        AutoBuilder.configure(
            robotState.getLatestFieldToRobot()::getValue,
            this::resetOdometry,
            robotState::getLatestFusedFieldRelativeChassisSpeeds,
            (speeds, feedforwards) -> applyPathPlannerRequest(speeds, feedforwards.robotRelativeForcesX(), feedforwards.robotRelativeForcesY()),
            new PPHolonomicDriveController(
                new PIDConstants(2.5, 0, 0), 
                new PIDConstants(4.0, 0, 0)
            ), 
            DrivetrainConstants.kPathPlannerRobotConfiguration,
            robotState::isRedAlliance,
            this
        );

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            robotState.setTrajectoryTargetPose(pose);
            Logger.recordOutput("PathPlanner/TargetPose", pose);
        });

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            robotState.setTrajectoryCurrentPose(pose);
            Logger.recordOutput("PathPlanner/CurrentPose", pose);
        });

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("PathPlanner/ActivePath", activePath.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetChassisSpeedsCallback((chassisSpeeds) -> {
            Logger.recordOutput("PathPlanner/TargetChassisSpeeds", chassisSpeeds);
        });
    }

    public void applyPathPlannerRequest(ChassisSpeeds speeds, Force[] forcesX, Force[] forcesY) {
        setControl(pathplannerRequest
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
            .withSpeeds(speeds)
            .withWheelForceFeedforwardsX(forcesX)
            .withWheelForceFeedforwardsY(forcesY)
        );
    }

    public void holdXStance() {
        setControl(brakeRequest);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this).withName("Swerve Request!");
    }

    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }

    public void resetOdometry(Pose2d pose) {
        io.resetOdometry(pose);
    }

    public void addVisionMeasurement(WeightedPoseEstimate weightedFieldPoseEstimate) {
        io.addVisionMeasurement(weightedFieldPoseEstimate);
    }

    public void setStateStandardDeviations(double xStd, double yStd, double rotStd) {
        io.setStateStandardDeviations(xStd, yStd, rotStd);
    }
    public void configureStandardDevsForDisabled() {
        setStateStandardDeviations(DrivetrainConstants.kDisabledDriveXStdDev, DrivetrainConstants.kDisabledDriveYStdDev, DrivetrainConstants.kDisabledDriveRotStdDev);
    }

    public void configureStandardDevsForEnabled() {
        setStateStandardDeviations(DrivetrainConstants.kEnabledDriveXStdDev, DrivetrainConstants.kEnabledDriveYStdDev, DrivetrainConstants.kEnabledDriveRotStdDev);
    }

    public SwerveDriveSimulation getSimulatedDrivetrain() {
        if (io instanceof DrivetrainIOSimulation) {
            return ((DrivetrainIOSimulation) io).getSimulatedDrivetrain();
        }

        return null;
    }
}
