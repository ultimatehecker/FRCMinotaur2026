package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Timer;

import frc.minolib.vision.CameraConfiguration;
import frc.minolib.vision.PhotonFiducialResult;
import frc.minolib.vision.PoseObservationType;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected PhotonPoseEstimator photonEstimator;
  private final List<PhotonFiducialResult> observations = new ArrayList<>();
  Set<Short> tagIds = new HashSet<>();

  public VisionIOPhotonVision(final CameraConfiguration cameraConfiguration, final AprilTagFieldLayout layout) {
    camera = new PhotonCamera(cameraConfiguration.getCameraName());
    photonEstimator = new PhotonPoseEstimator(layout, cameraConfiguration.getTransformOffset());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    observations.clear();
    tagIds.clear();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);

      if (visionEstimate.isEmpty()) {
        visionEstimate = photonEstimator.estimateAverageBestTargetsPose(result);
      }

      visionEstimate.ifPresent(estimate -> {
            long tagsSeenBitMap = 0;
            double averageAmbiguity = 0.0;
            double averageTagDistance = 0.0;

            for (int i = 0; i < estimate.targetsUsed.size(); i++) {
              tagsSeenBitMap |= 1L << estimate.targetsUsed.get(i).getFiducialId();
              averageAmbiguity += estimate.targetsUsed.get(i).getPoseAmbiguity();
              averageTagDistance += estimate.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getNorm();

              tagIds.add((short) estimate.targetsUsed.get(i).getFiducialId());
            }

            averageAmbiguity /= estimate.targetsUsed.size();
            averageTagDistance /= estimate.targetsUsed.size();

            observations.add(
                new PhotonFiducialResult(
                    result.getTimestampSeconds(),
                    estimate.estimatedPose,
                    Timer.getFPGATimestamp() - result.getTimestampSeconds(),
                    averageAmbiguity,
                    result.multitagResult.isPresent() ? result.multitagResult.get().estimatedPose.bestReprojErr : 0.0,
                    tagsSeenBitMap,
                    estimate.targetsUsed.size(),
                    averageTagDistance,
                    estimate.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ? PoseObservationType.MULTI_TAG : PoseObservationType.SINGLE_TAG
                )
            );
        });
    }
    inputs.poseObservations = observations.toArray(new PhotonFiducialResult[0]);

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;

    for (int id : tagIds) {
        inputs.tagIds[i++] = id;
    }
  }
}