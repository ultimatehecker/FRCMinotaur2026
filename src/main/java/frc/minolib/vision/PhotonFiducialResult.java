package frc.minolib.vision;


import edu.wpi.first.math.geometry.Pose3d;
//import frc.robot.subsystems.vision.VisionIO;

/** A data class for a fiducial result. */
public record PhotonFiducialResult(
    int numberOfFiducials,
    double poseAmbiguity,
    Pose3d cameraPose,
    double averageDistance,
    double reprojectionError,
    long tagsSeenBitMap,
    double timestamp,
    double latencySeconds
    //VisionIO.PoseObservationType type
) 
{

}