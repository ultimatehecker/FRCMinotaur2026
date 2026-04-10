package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.swerve.pathplanner.PathPlannerLogging;

import frc.robot.RobotState;
import frc.robot.constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
    private DrivetrainIO io;
    private RobotState robotState;

    private DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

    private ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    private ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    private ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
    private ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

    private final Object moduleIOLock = new Object();

    private RobotConfig robotConfig;

    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final ApplyRobotSpeeds pathplannerRequest = new ApplyRobotSpeeds()
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withDesaturateWheelSpeeds(true);

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    private final SysIdRoutine translationSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            Seconds.of(5),
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> io.setControl(translationCharacterization.withVolts(output)), null, this
        )
    );

    private final SysIdRoutine rotationSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
                Volts.of(Math.PI / 6).per(Second),
                Volts.of(Math.PI),
                Seconds.of(5),
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                io.setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private final SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(volts -> io.setControl(steerCharacterization.withVolts(volts)), null, this)
    );

    public enum SysIdMechanism {
        SWERVE_TRANSLATION,
        SWERVE_ROTATION,
        SWERVE_STEER
    }

    public Drivetrain(DrivetrainIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        configurePathPlanner();
    }

    @Override
    public void periodic() {
        io.updateDrivetrainInputs(inputs);
        Logger.processInputs("Drivetrain", inputs);

        robotState.incrementIterationCount();
        if (DriverStation.isDisabled()) {
            configureStandardDevsForDisabled();
        } else {
            configureStandardDevsForEnabled();
        }

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                io.setOperatorPerspectiveForward(allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        LoggedTracer.record("DrivetrainPeriodic");
    }

    private void configurePathPlanner() {
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            () -> robotState.getLatestFieldToRobot().getValue(),
            this::resetOdometry,
            robotState::getLatestDesiredRobotRelativeChassisSpeeds,
            (speeds, feedforwards) -> applyPathPlannerRequest(speeds, feedforwards.robotRelativeForcesX(), feedforwards.robotRelativeForcesY()),
            new PPHolonomicDriveController(
                new PIDConstants(DrivetrainConstants.kPathPlannerDriveHolonomicControllerP, DrivetrainConstants.kPathPlannerDriveHolonomicControllerI, DrivetrainConstants.kPathPlannerDriveHolonomicControllerD), 
                new PIDConstants(DrivetrainConstants.kPathPlannerSteerHolonomicControllerP, DrivetrainConstants.kPathPlannerSteerHolonomicControllerI, DrivetrainConstants.kPathPlannerSteerHolonomicControllerD)
            ),
            robotConfig,
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

    public Command sysIdQuasistatic(SysIdMechanism mechanism, SysIdRoutine.Direction direction) {
        final SysIdRoutine routine = switch (mechanism) {
            case SWERVE_TRANSLATION -> translationSysIdRoutine;
            case SWERVE_ROTATION -> rotationSysIdRoutine;
            case SWERVE_STEER -> steerSysIdRoutine;
            default -> throw new IllegalArgumentException(String.format("Mechanism %s is not supported.", mechanism));
        };

        return routine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdMechanism mechanism, SysIdRoutine.Direction direction) {
        final SysIdRoutine routine = switch (mechanism) {
            case SWERVE_TRANSLATION -> translationSysIdRoutine;
            case SWERVE_ROTATION -> rotationSysIdRoutine;
            case SWERVE_STEER -> steerSysIdRoutine;
            default -> throw new IllegalArgumentException(String.format("Mechanism %s is not supported.", mechanism));
        };

        return routine.dynamic(direction);
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
