package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public record ShootingParameters(
      boolean isValid, /** heading the robot must hold */
      Rotation2d driveAngle, /** rate-of-change of driveAngle (rad/s) */
      double driveVelocity, /** target hood angle (radians) */
      double hoodAngle, /** rate-of-change of hoodAngle (rad/s) */
      double hoodVelocity, /** target flywheel speed (rpm) */
      double flywheelVelocity, /** launcher to target with lookahead */
      double distance, /** launcher to target without lookahead */
      double distanceNoLookahead, /** solved TOF (seconds) */
      double timeOfFlight, /** true = passing profile */
      boolean passing, /** 0-100 shoot-readiness score */
      double confidence, /** Newton iterations (for telemetry) */
      int iterationsUsed
) {
      public static final ShootingParameters INVALID = new ShootingParameters(false, new Rotation2d(), 0, 0, 0, 0, 0, 0, 0, false, 0, 0);
}
