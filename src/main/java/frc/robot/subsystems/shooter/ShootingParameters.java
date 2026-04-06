package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public record ShootingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing,
      double confidence,
      int iterationsUsed
) {
      public static final ShootingParameters INVALID = new ShootingParameters(false, new Rotation2d(), 0, 0, 0, 0, 0, 0, 0, false, 0, 0);
}