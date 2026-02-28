package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.utilities.SubsystemDataProcessor;

public interface DrivetrainIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class DrivetrainIOInputs extends SwerveDriveState {
        public double gyroAngle = 0.0;

        public DrivetrainIOInputs() {
            this.Pose = Pose2d.kZero;
        }

        public void logState(SwerveDrivetrain.SwerveDriveState state) {
            this.Pose = state.Pose;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.Speeds = state.Speeds;
            this.OdometryPeriod = state.OdometryPeriod;
        }
    }

    @AutoLog
    public class ModuleIOInputs {
        public boolean isDriveConnected = false;
        public double driveSupplyCurrentAmperes = 0.0;
        public double driveStatorCurrentAmperes = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveTemperatureCelsius = 0.0;

        public boolean isSteerConnected = false;
        public double steerSupplyCurrentAmperes = 0.0;
        public double steerStatorCurrentAmperes = 0.0;
        public double steerAppliedVoltage = 0.0;
        public double steerTemperatureCelsius = 0.0;
    }

    public void updateDrivetrainInputs(DrivetrainIOInputs inputs);

    public void updateModuleInputs(ModuleIOInputs... inputs);

    public void setControl(SwerveRequest request);

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    public void setBrakeMode(boolean enable);

    public default void updateSimulationState() {};

    public void setStateStandardDeviations(double xStandardDeviations, double yStandardDeviations, double rotationStandardDeviations);

    public void addVisionMeasurement(WeightedPoseEstimate visionFieldPoseEstimate); // do later because i dont feel like it

    public void resetOdometry(Pose2d pose);

    @Override
    public void refreshData(); 
}
