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

    public static final String kBackLeftCameraName = "bL";
    public static final String kBackRightCameraName = "bR";

    public static final CameraConfiguration kBackLeftConfiguration = new CameraConfiguration()
            .withCameraName(kBackLeftCameraName)
            .withCameraLocation(CameraLocation.BACK_LEFT)
            .withLengthOffset(Inches.of(9.2837556))
            .withWidthOffset(Inches.of(1.6423085))
            .withHeightOffset(Inches.of(6.9584678))
            .withMountingPitch(Degrees.of(337))
            .withMountingYaw(Degrees.of(328));

    public static final CameraConfiguration kBackRightConfiguration = new CameraConfiguration()
            .withCameraName(kBackRightCameraName)
            .withCameraLocation(CameraLocation.BACK_RIGHT)
            .withLengthOffset(Inches.of(9.2837556))
            .withWidthOffset(Inches.of(-2.1438256))
            .withHeightOffset(Inches.of(6.9584678))
            .withMountingPitch(Degrees.of(340))
            .withMountingYaw(Degrees.of(32));

    public static double kMaximumTagAmbiguity = 0.15;
    public static Distance kMaximumZPoseError = Inches.of(0.5);

    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
    };

    public static double linearStdDevMegatag2Factor = 0.5; 
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; 

    public static final double kSimAverageErrorPixels = 0.1;
    public static final double kSimErrorStdDevPixels = 0.05;
}
