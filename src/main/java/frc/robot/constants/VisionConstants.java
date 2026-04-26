package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;
import frc.minolib.vision.CameraConfiguration;
import frc.minolib.vision.CameraConfiguration.CameraLocation;

public class VisionConstants {
    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final String kBackLeftCameraName = "shooterbL";
    public static final String kBackRightCameraName = "shooterbR";

    public static final CameraConfiguration kBackLeftConfiguration = new CameraConfiguration()
            .withCameraName(kBackLeftCameraName)
            .withCameraLocation(CameraLocation.BACK_LEFT)
            .withLengthOffset(Inches.of(-13.169117291))
            .withWidthOffset(Inches.of(5.947719578))
            .withHeightOffset(Inches.of(11.063614653))
            .withMountingPitch(Degrees.of(345))
            .withMountingYaw(Degrees.of(320));

    public static final CameraConfiguration kBackRightConfiguration = new CameraConfiguration()
            .withCameraName(kBackRightCameraName)
            .withCameraLocation(CameraLocation.BACK_RIGHT)
            .withLengthOffset(Inches.of(-13.169117291))
            .withWidthOffset(Inches.of(-5.947719578))
            .withHeightOffset(Inches.of(11.063614653))
            .withMountingPitch(Degrees.of(345))
            .withMountingYaw(Degrees.of(220));

    public static double kMaximumTagAmbiguity = 0.1;
    public static Distance kMaximumZPoseError = Inches.of(0.5);

    public static double linearStdDevBaseline = 0.0254; // Meters
    public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
    };

    public static final double kSimAverageErrorPixels = 0.1;
    public static final double kSimErrorStdDevPixels = 0.05;
}
