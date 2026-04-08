package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.vision.PhotonFiducialResult;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        boolean connected;
        public int[] tagIds = new int[0];
        PhotonFiducialResult[] poseObservations = new PhotonFiducialResult[0];
    }

    default void updateInputs(VisionIOInputs inputs) {}
}