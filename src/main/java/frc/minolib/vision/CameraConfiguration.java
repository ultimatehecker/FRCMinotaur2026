package frc.minolib.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public class CameraConfiguration {
    public enum CameraLocation {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        NONE
    }

    private String cameraName;
    public double cameraMountingRollRadians = 0;
    public double cameraMountingYawRadians = 0;
    public double cameraMountingPitchRadians = 0;
    public double cameraHeightOffsetMeters = 0;
    public double cameraLengthOffsetMeters = 0;
    public double cameratWidthOffsetMeters = 0;
    public CameraLocation cameraLocation = CameraLocation.NONE;

    public CameraConfiguration(String cameraName) {
        this.cameraName = cameraName;
    }

    public CameraConfiguration() {
        this("le");
    }

    public String getName() {
        return cameraName;
    }

    public CameraConfiguration withCameraName(String name) {
        this.cameraName = name;
        return this;
    }

    public CameraConfiguration withMountingRoll(double rollRadians) {
        this.cameraMountingRollRadians = rollRadians;
        return this;
    }

    public CameraConfiguration withMountingYaw(double yawRadians) {
        this.cameraMountingYawRadians = yawRadians;
        return this;
    }

    public CameraConfiguration withMountingPitch(double pitchRadians) {
        this.cameraMountingPitchRadians = pitchRadians;
        return this;
    }

    public CameraConfiguration withHeightOffset(double heightOffsetMeters) {
        this.cameraHeightOffsetMeters = heightOffsetMeters;
        return this;
    }

    public CameraConfiguration withLengthOffset(double lengthOffsetMeters) {
        this.cameraLengthOffsetMeters = lengthOffsetMeters;
        return this;
    }

    public CameraConfiguration withWidthOffset(double widthOffsetMeters) {
        this.cameratWidthOffsetMeters = widthOffsetMeters;
        return this;
    }

    public CameraConfiguration withCameraLocation(CameraLocation cameraLocation) {
        this.cameraLocation = cameraLocation;
        return this;
    }

    public String getCameraName() {
        return this.cameraName;
    }

    public Translation3d getTranslationOffset() {
        return new Translation3d(this.cameraLengthOffsetMeters, this.cameraHeightOffsetMeters, this.cameratWidthOffsetMeters);
    }

    public Rotation3d getRotationOffset() {
        return new Rotation3d(this.cameraMountingRollRadians, this.cameraMountingPitchRadians, this.cameraMountingYawRadians);
    }

    public Transform3d getTransformOffset() {
        return new Transform3d(
            new Translation3d(this.cameraLengthOffsetMeters, this.cameraHeightOffsetMeters, this.cameratWidthOffsetMeters), 
            new Rotation3d(this.cameraMountingRollRadians, this.cameraMountingPitchRadians, this.cameraMountingYawRadians)
        );
    }

    public Translation2d getTranslationToRobotCenter() {
        return new Translation2d(this.cameraLengthOffsetMeters, this.cameratWidthOffsetMeters);
    }

    public CameraLocation getLocation() {
        return cameraLocation;
    }
}
