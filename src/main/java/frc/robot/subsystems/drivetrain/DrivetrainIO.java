package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DrivetrainIO {
    @AutoLog
    public class DrivetrainIOInputs {
        public Pose2d pose = new Pose2d();
        public ChassisSpeeds measuredRobotRelativeChassisSpeeds = new ChassisSpeeds();
        public ChassisSpeeds referenceRobotRelativeChassisSpeeds = new ChassisSpeeds();
        public ChassisSpeeds measuredFieldRelativeChassisSpeeds = new ChassisSpeeds();
        public ChassisSpeeds referenceFieldRelativeChassisSpeeds = new ChassisSpeeds();
        public SwerveModuleState[] currentModuleStates;
        public SwerveModuleState[] referenceModuleStates;
        public SwerveModulePosition[] modulePositions;
        public Rotation2d rawHeading = new Rotation2d();
        public double timestamp;
        public double odometryPeriod;
        public int successfulDaqs;
        public int failedDaqs;

        public void logState(SwerveDrivetrain.SwerveDriveState state) {
            this.pose = state.Pose;
            this.currentModuleStates = state.ModuleStates;
            this.referenceModuleStates = state.ModuleTargets;
            this.modulePositions = state.ModulePositions;
            this.rawHeading = state.RawHeading;
            this.timestamp = state.Timestamp;
            this.odometryPeriod = state.OdometryPeriod;
            this.successfulDaqs = state.SuccessfulDaqs;
            this.failedDaqs = state.FailedDaqs;
        }
    }

    @AutoLog
    public class ModuleIOInputs {
        public boolean isDriveConnected = false;
        public double driveSupplyCurrentAmperes = 0.0;
        public double driveStatorCurrentAmperes = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveTempuratureCelsius = 0.0;

        public boolean isSteerConnected = false;
        public double steerSupplyCurrentAmperes = 0.0;
        public double steerStatorCurrentAmperes = 0.0;
        public double steerAppliedVoltage = 0.0;
        public double steerTempuratureCelsius = 0.0;
    }

    public void updateDrivetrainInputs(DrivetrainIOInputs inputs);

    public void updateModuleInputs(ModuleIOInputs... inputs);

    public void setControl(SwerveRequest request);

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    public void setCOF(Translation2d cor);

    public Translation2d getCOR();

    public void setBrakeMade();

    public void setTargetChassisSpeeds(ChassisSpeeds targetChassisSpeeds);

    public default void updateSimulationState() {};

    public void setStateStandardDeviations(double xStandardDeviations, double yStandardDeviations, double rotationStandardDeviations);

    public void addVisionMeasurement(); // do later because i dont feel like it

    public void resetOdometry(Pose2d pose);

    // irl do this but again too lazy
    /*
     * @Override
     * public void refreshData
     */
}
