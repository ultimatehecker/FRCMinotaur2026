package frc.robot.subsystems.drivetrain;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.swerve.pathplanner.PathPlannerLogging;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.minolib.wpilib.RobotTime;
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

    private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    private final ApplyRobotSpeeds pathplannerAutoRequest = new ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withDesaturateWheelSpeeds(true);

    public Drivetrain(DrivetrainIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (moduleIOLock) {
                io.updateModuleInputs(frontLeftInputs, frontRightInputs, backLeftInputs, backRightInputs);
            }
        }, io);

        //configurePathPlanner();
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.updateDrivetrainInputs(inputs);

        synchronized (moduleIOLock) {
            Logger.processInputs("Drivetrain", inputs);
            Logger.processInputs("Drivetrain/Module Data/Front Left", frontLeftInputs);
            Logger.processInputs("Drivetrain/Module Data/Front Right", frontRightInputs);
            Logger.processInputs("Drivetrain/Module Data/Back Left", backLeftInputs);
            Logger.processInputs("Drivetrain/Module Data/Back Right", backRightInputs);
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
        ModuleConfig moudleConfiguration = new ModuleConfig(
            DrivetrainConstants.getModuleConstants()[0].WheelRadius,
            DrivetrainConstants.kMaximumLinearVelocityMetersPerSecond,
            DrivetrainConstants.kWheelCOF,
            DCMotor.getKrakenX60Foc(1),
            DrivetrainConstants.getModuleConstants()[0].DriveMotorGearRatio,
            DrivetrainConstants.getModuleConstants()[0].SlipCurrent,
            1
        );

        RobotConfig robotConfiguration = new RobotConfig(
            DrivetrainConstants.kRobotMassKilograms,
            DrivetrainConstants.kRobotMOI,
            moudleConfiguration,
            new Translation2d(0.31115, 0.31115),
            new Translation2d(0.31115, -0.31115),
            new Translation2d(-0.31115, 0.31115),
            new Translation2d(-0.31115, -0.31115)
        );

        AutoBuilder.configure(
            () -> robotState.getLatestFieldToRobot().getValue(),
            this::resetOdometry,
            () -> robotState.getLatestFusedRobotRelativeChassisSpeed(),
            controller,
            robotConfiguration,
            () -> robotState.isRedAlliance(),
            this
        );

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            robotState.setTrajectoryTargetPose(pose);
            Logger.recordOutput("PathPlanner/targetPose", pose);
        });

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            robotState.setTrajectoryCurrentPose(pose);
            Logger.recordOutput("PathPlanner/currentPose", pose);
        });

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("PathPlanner/activePath", activePath.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetChassisSpeedsCallback((chassisSpeeds) -> {
            Logger.recordOutput("PathPlanner/targetChassisSpeeds", chassisSpeeds);
        });
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
