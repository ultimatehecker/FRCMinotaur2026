package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.localization.WeightedPoseEstimate;
import frc.robot.RobotState;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final VisionIO[] io;
    private final RobotState robotState;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    List<WeightedPoseEstimate> allPoseEstimates = new ArrayList<>();
    double visionConfidence = 0.0;

    public Vision(RobotState robotState, VisionIO... io) {
        this.robotState = robotState;
        this.io = io;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        allPoseEstimates.clear();

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = VisionConstants.kAprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean rejectPose = observation.numTags() == 0 || (observation.numTags() == 1 && observation.averageAmbiguity() > VisionConstants.kMaximumTagAmbiguity) 
                    || observation.cameraPose().getX() < 0.0
                    || observation.cameraPose().getX() > VisionConstants.kAprilTagLayout.getFieldLength() // TODO: Switch these to FieldConstants when they are imported
                    || observation.cameraPose().getY() < 0.0
                    || observation.cameraPose().getY() > VisionConstants.kAprilTagLayout.getFieldWidth();

                robotPoses.add(observation.cameraPose());

                if (rejectPose) {
                    robotPosesRejected.add(observation.cameraPose());
                } else {
                    robotPosesAccepted.add(observation.cameraPose());
                }

                if (rejectPose) {
                    continue;
                }

                double linearStdDev = VisionConstants.linearStdDevBaseline * (Math.pow(observation.averageTagDistance(), 2.0) / observation.numTags());
                
                if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
                    linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
                }
                
                allPoseEstimates.add(new WeightedPoseEstimate(
                    observation.cameraPose().toPose2d(),
                    observation.timestamp(),
                    VecBuilder.fill(
                        linearStdDev,   
                        linearStdDev,   
                        VisionConstants.angularStdDevBaseline  
                    ),
                    observation.numTags()
                ));
            }

            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

        List<WeightedPoseEstimate> acceptedThisCycle = new ArrayList<>();
        Map<Double, List<WeightedPoseEstimate>> grouped = groupByTimestamp(allPoseEstimates, 0.02);

        for (var group : grouped.values()) {
            WeightedPoseEstimate fused = group.size() == 1 ? group.get(0) : fuseObservations(group);

            acceptedThisCycle.add(fused);
            robotState.updateVisionPoseEstimate(fused);
        }

        computeVisionConfidence(acceptedThisCycle);
        robotState.setVisionConfidence(visionConfidence);

        Logger.recordOutput("Vision/Confidence", visionConfidence);
    }

    private Map<Double, List<WeightedPoseEstimate>> groupByTimestamp(List<WeightedPoseEstimate> observations, double tolerance) {
        Map<Double, List<WeightedPoseEstimate>> groups = new HashMap<>();
        
        for (var obs : observations) {
            double bucket = Math.round(obs.getTimestampSeconds() / 0.02) * 0.02;
            groups.computeIfAbsent(bucket, k -> new ArrayList<>()).add(obs);
        }
        
        return groups;
    }

    private WeightedPoseEstimate fuseObservations(List<WeightedPoseEstimate> observations) {
        if (observations.size() == 1) {
            return observations.get(0);
        }
        
        observations.sort(Comparator.comparingDouble(WeightedPoseEstimate::getTimestampSeconds));
        double targetTimestamp = observations.get(observations.size() - 1).getTimestampSeconds();
        
        // Project all poses to target timestamp using odometry
        List<Pose2d> alignedPoses = new ArrayList<>();
        List<Matrix<N3, N1>> alignedStdDevs = new ArrayList<>();
        
        for (var obs : observations) {
            Pose2d alignedPose;
            
            if (Math.abs(obs.getTimestampSeconds() - targetTimestamp) < 0.001) {
                // Already at target time (within 1ms)
                alignedPose = obs.getVisionRobotPoseMeters();
            } else {
                // Need to project forward in time
                var poseAtObsTime = robotState.getFieldToRobot(obs.getTimestampSeconds());
                var poseAtTargetTime = robotState.getFieldToRobot(targetTimestamp);
                
                if (poseAtObsTime.isEmpty() || poseAtTargetTime.isEmpty()) {
                    continue;
                }
                
                Transform2d motion = poseAtTargetTime.get().minus(poseAtObsTime.get());
                
                alignedPose = obs.getVisionRobotPoseMeters().transformBy(motion);
            }
            
            alignedPoses.add(alignedPose);
            alignedStdDevs.add(obs.getVisionMeasurementStdDevs());
        }
        
        if (alignedPoses.isEmpty()) {
            // Couldn't align any poses - return newest observation as fallback
            return observations.get(observations.size() - 1);
        }
        
        // Now fuse the temporally-aligned poses
        double weightedX = 0;
        double weightedY = 0;
        double totalWeightX = 0;
        double totalWeightY = 0;
        
        // For rotation: track if we should fuse it or just pick one
        boolean hasValidRotation = false;
        double weightedCos = 0;
        double weightedSin = 0;
        double totalRotWeight = 0;
        
        for (int i = 0; i < alignedPoses.size(); i++) {
            Pose2d pose = alignedPoses.get(i);
            Matrix<N3, N1> stdDevs = alignedStdDevs.get(i);
            
            double xStdDev = stdDevs.get(0, 0);
            double yStdDev = stdDevs.get(1, 0);
            double rotStdDev = stdDevs.get(2, 0);
            
            // X and Y fusion (always valid)
            double xVariance = xStdDev * xStdDev;
            double yVariance = yStdDev * yStdDev;
            double xWeight = 1.0 / xVariance;
            double yWeight = 1.0 / yVariance;
            
            weightedX += pose.getX() * xWeight;
            weightedY += pose.getY() * yWeight;
            totalWeightX += xWeight;
            totalWeightY += yWeight;
            
            // Rotation fusion (only if stdDev is reasonable)
            if (rotStdDev < 100.0) { // Arbitrary threshold - adjust as needed
                hasValidRotation = true;
                double rotVariance = rotStdDev * rotStdDev;
                double rotWeight = 1.0 / rotVariance;
                
                // Use cos/sin to handle angle wraparound properly
                weightedCos += pose.getRotation().getCos() * rotWeight;
                weightedSin += pose.getRotation().getSin() * rotWeight;
                totalRotWeight += rotWeight;
            }
        }
        
        // Calculate fused X and Y
        double fusedX = weightedX / totalWeightX;
        double fusedY = weightedY / totalWeightY;
        
        // Calculate fused rotation
        Rotation2d fusedRotation;
        double fusedRotStdDev;
        
        if (hasValidRotation && totalRotWeight > 0) {
            // Fuse rotations using trigonometry
            fusedRotation = new Rotation2d(weightedCos, weightedSin);
            fusedRotStdDev = Math.sqrt(1.0 / totalRotWeight);
        } else {
            // No valid rotation data - use newest pose's rotation
            fusedRotation = alignedPoses.get(alignedPoses.size() - 1).getRotation();
            fusedRotStdDev = Double.POSITIVE_INFINITY; // Don't trust it
        }
        
        // Build fused standard deviations
        Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(
            Math.sqrt(1.0 / totalWeightX),
            Math.sqrt(1.0 / totalWeightY),
            fusedRotStdDev
        );
        
        // Sum total tags
        int totalTags = observations.stream()
            .mapToInt(WeightedPoseEstimate::getNumTags)
            .sum();
        
        return new WeightedPoseEstimate(
            new Pose2d(fusedX, fusedY, fusedRotation),
            targetTimestamp,
            fusedStdDevs,
            totalTags
        );
    }

    public Optional<WeightedPoseEstimate> getBestSingleEstimate() {
        WeightedPoseEstimate best = null;
        double bestAmbiguity = Double.MAX_VALUE;

        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean rejected = observation.numTags() == 0 || (observation.numTags() == 1 && observation.averageAmbiguity() > VisionConstants.kMaximumTagAmbiguity)
                    || observation.cameraPose().getX() < 0.0
                    || observation.cameraPose().getX() > VisionConstants.kAprilTagLayout.getFieldLength()
                    || observation.cameraPose().getY() < 0.0
                    || observation.cameraPose().getY() > VisionConstants.kAprilTagLayout.getFieldWidth();

                if (rejected) continue;

                double ambiguity = observation.numTags() > 1 ? 0.0 : observation.averageAmbiguity();

                if (ambiguity < bestAmbiguity) {
                    bestAmbiguity = ambiguity;

                    double linearStdDev = VisionConstants.linearStdDevBaseline * (Math.pow(observation.averageTagDistance(), 2.0) / observation.numTags());

                    if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
                        linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
                    }

                    best = new WeightedPoseEstimate(
                        observation.cameraPose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, VisionConstants.angularStdDevBaseline),
                        observation.numTags());
                }
            }
        }

        Logger.recordOutput("Vision/BestSingleEstimateAmbiguity", bestAmbiguity == Double.MAX_VALUE ? -1.0 : bestAmbiguity);
        Logger.recordOutput("Vision/BestSingleEstimateAvailable", best != null);

        return Optional.ofNullable(best);
    }

    private void computeVisionConfidence(List<WeightedPoseEstimate> acceptedEstimates) {
        if (acceptedEstimates.isEmpty()) {
            // No accepted observations this cycle — decay toward zero
            visionConfidence = Math.max(0.0, visionConfidence - 0.05);
            return;
        }

        double totalConfidence = 0.0;
        double totalWeight = 0.0;

        for (var estimate : acceptedEstimates) {
            double tagFactor = Math.min(estimate.getNumTags() / 3.0, 1.0);
            double xStdDev = estimate.getVisionMeasurementStdDevs().get(0, 0);
            double distanceFactor = MathUtil.clamp(1.0 - (xStdDev / VisionConstants.linearStdDevBaseline) * 0.1, 0.0, 1.0);
            double observationConfidence = 0.6 * tagFactor + 0.4 * distanceFactor;

            totalConfidence += observationConfidence * estimate.getNumTags();
            totalWeight += estimate.getNumTags();
        }

        double rawConfidence = totalWeight > 0 ? totalConfidence / totalWeight : 0.0;

        visionConfidence = 0.8 * visionConfidence + 0.2 * rawConfidence;
        visionConfidence = MathUtil.clamp(visionConfidence, 0.0, 1.0);
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}