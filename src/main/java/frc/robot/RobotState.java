package frc.robot;

import java.lang.StackWalker.Option;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.math.ConcurrentTimeInterpolatableBuffer;
import frc.robot.constants.GlobalConstants;

public class RobotState {
    private final Consumer<WeightedPoseEstimate> visionEstimateConsumer;

    public RobotState(final Consumer<WeightedPoseEstimate> visionEsimateConsumer) {
        this.visionEstimateConsumer = visionEsimateConsumer;
        fieldToRobot.addSample(0.0, Pose2d.kZero);
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }

    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot = ConcurrentTimeInterpolatableBuffer.createBuffer(GlobalConstants.kLoopBackTimeSeconds);

    private double lastUsedVisionEstimateTimestamp = 0.0;
    private Pose2d lastUsedVisionPoseEstimate = Pose2d.kZero;

    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());

    private final AtomicInteger iteration = new AtomicInteger(0);

    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularPosition = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularPosition = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);

    private final ConcurrentTimeInterpolatableBuffer<Double> driveAccelerationX = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveAccelerationY = ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(GlobalConstants.kLoopBackTimeSeconds);

    private final AtomicBoolean enablePathCancel = new AtomicBoolean(false);

    private double autoStartTime;

    private Optional<Pose2d> trajectoryTargetPose = Optional.empty();
    private Optional<Pose2d> trajectoryCurrentPose = Optional.empty();

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel.set(true);
    }

    public void disablePathCancel() {
        enablePathCancel.set(false);
    }

    public boolean getPathCancel() {
        return enablePathCancel.get();
    }

    public void incrementIterationCount() {
        iteration.incrementAndGet();
    }

    public int getIteration() {
        return iteration.get();
    }

    public IntSupplier getIterationSupplier() {
        return () -> getIteration();
    }

    public void setTrajectoryTargetPose(Pose2d pose) {
        trajectoryTargetPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryTargetPose() {
        return trajectoryTargetPose;
    }

    public void setTrajectoryCurrentPose(Pose2d pose) {
        trajectoryCurrentPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryCurrentPose() {
        return trajectoryCurrentPose;
    }

    public double getDrivePitchRadians() {
        if (this.drivePitchAngularPosition.getInternalBuffer().lastEntry() != null) {
            return drivePitchAngularPosition.getInternalBuffer().lastEntry().getValue();
        }

        return 0.0;
    }

    public double getDriveRollRadians() {
        if (this.driveRollAngularPosition.getInternalBuffer().lastEntry() != null) {
            return driveRollAngularPosition.getInternalBuffer().lastEntry().getValue();
        }

        return 0.0;
    }

    public void addDriveMotionMeasurements(
        double timestamp, 
        double yawAngularVelocity,
        double pitchAngularVelocity,
        double rollAngularVelocity,
        double pitchAngularPosition,
        double rollAngularPostiion,
        double accelerationX,
        double accelerationY,
        ChassisSpeeds measuredRobotRelativeChassisSpeeds,
        ChassisSpeeds measuredFieldRelativeChassisSpeeds,
        ChassisSpeeds desiredRobotRelativeChassisSpeeds,
        ChassisSpeeds desiredFieldRelativeChassisSpeeds,
        ChassisSpeeds fusedFieldRelativeChasssisSpeeds
    ) {
        driveYawAngularVelocity.addSample(timestamp, yawAngularVelocity);
        drivePitchAngularVelocity.addSample(timestamp, pitchAngularVelocity);
        driveRollAngularVelocity.addSample(timestamp, rollAngularVelocity);
        drivePitchAngularPosition.addSample(timestamp, pitchAngularPosition);
        driveRollAngularPosition.addSample(timestamp, rollAngularPostiion);

        driveAccelerationX.addSample(timestamp, accelerationX);
        driveAccelerationY.addSample(timestamp, accelerationY);

        this.measuredRobotRelativeChassisSpeeds.set(measuredRobotRelativeChassisSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeChassisSpeeds);
        this.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeChassisSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeChasssisSpeeds);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTimesSeconds) {
        var potentiallyFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = potentiallyFieldToRobot == null ? Pose2d.kZero : potentiallyFieldToRobot.getValue();

        var dt = getLatestMeasuredRobotRelativeChassisSpeeds();
        dt = dt.times(lookaheadTimesSeconds);

        return fieldToRobot.exp(new Twist2d(dt.vxMetersPerSecond, dt.vyMetersPerSecond, dt.omegaRadiansPerSecond));
    }

    public ChassisSpeeds getLatestMeasuredRobotRelativeChassisSpeeds() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
        return desiredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeeds() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeeds() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestMeasuredRobotRelativeChassisSpeeds();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeeds().omegaRadiansPerSecond;

        return speeds;
    }

    private Optional<Double> getMaxAbsoluteValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().max(Double::compare);

        if(max.isEmpty() || min.isEmpty()) {
            return Optional.empty();
        }

        if(Math.abs(max.get()) >= Math.abs(min.get())) return max;
        else return min;
    }

    public Optional<Double> getMaximumAbsoluteDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        if(Robot.isReal()) {
            return getMaxAbsoluteValueInRange(driveYawAngularVelocity, minTime, maxTime);
        }

        return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    }

    public Optional<Double> getMaximumAbsoluteDrivePitchAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsoluteValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaximumAbsoluteDriveRollAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsoluteValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updateVisionPoseEstimate(WeightedPoseEstimate weightedPoseEstimate) {
        lastUsedVisionEstimateTimestamp = weightedPoseEstimate.getTimestampSeconds();
        lastUsedVisionPoseEstimate = weightedPoseEstimate.getVisionRobotPoseMeters();
        visionEstimateConsumer.accept(weightedPoseEstimate);
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }
}