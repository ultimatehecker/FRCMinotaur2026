package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.math.Bounds;
import frc.minolib.utilities.AllianceFlipUtility;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;

import lombok.Getter;

public class ShooterCalculator {
    /*
    private final RobotState robotState;

    private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleLUT = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap flywheelVelocityLUT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightLUT = new InterpolatingDoubleTreeMap();

    private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleLUT = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap passingFlywheelVelocityLUT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passingTimeOfFlightLUT = new InterpolatingDoubleTreeMap();

    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / GlobalConstants.kLoopPeriodSeconds));
    private final LinearFilter driveAngleFilter = LinearFilter.movingAverage((int) (0.1 / GlobalConstants.kLoopPeriodSeconds));

    private double flywheelVelocityOffset = 0.0;
    @Getter private double hoodAngleOffsetDegrees = 0.0;

    private double lastHoodAngle;
    private Rotation2d lastDriveAngle;

    private ShootingParameters latestParameters = null;

    private static final double minDistance;
    private static final double maxDistance;
    private static final double passingMinDistance;
    private static final double passingMaxDistance;
    private static final double phaseDelay;

    public static final double hubPresetDistance = 0.96;
    public static final double towerPresetDistance = 2.5;
    public static final double trenchPresetDistance = 3.03;
    public static final double outpostPresetDistance = 4.84;
    public static final double passingPresetDistance = 7.0;

    public static final ShootingPreset passingPreset;
    public static final ShootingPreset hubPreset;
    public static final ShootingPreset towerPreset;
    public static final ShootingPreset trenchPreset;
    public static final ShootingPreset outpostPreset;
    public static final ShootingPreset hoodMinPreset;
    public static final ShootingPreset hoodMaxPreset;

    private static final double xPassTarget = Units.inchesToMeters(37); //TODO: Convert to an actual Translation in constants
    private static final double yPassTarget = Units.inchesToMeters(65); //TODO: Convert to an actual Translation in constants

    static {
        minDistance = 0.9;
        maxDistance = 4.9;
        passingMinDistance = 5.4;
        passingMaxDistance = 17.16;
        phaseDelay = 0.03;

        hoodAngleLUT.put(0.96, Rotation2d.fromDegrees(10.0));
        hoodAngleLUT.put(1.16, Rotation2d.fromDegrees(12.0));
        hoodAngleLUT.put(1.58, Rotation2d.fromDegrees(14.0));
        hoodAngleLUT.put(2.07, Rotation2d.fromDegrees(18.5));
        hoodAngleLUT.put(2.37, Rotation2d.fromDegrees(22.0));
        hoodAngleLUT.put(2.47, Rotation2d.fromDegrees(23.0));
        hoodAngleLUT.put(2.70, Rotation2d.fromDegrees(24.0));
        hoodAngleLUT.put(2.94, Rotation2d.fromDegrees(25.0));
        hoodAngleLUT.put(3.48, Rotation2d.fromDegrees(27.0));
        hoodAngleLUT.put(3.92, Rotation2d.fromDegrees(32.0));
        hoodAngleLUT.put(4.35, Rotation2d.fromDegrees(34.0));
        hoodAngleLUT.put(4.84, Rotation2d.fromDegrees(38.0));

        flywheelVelocityLUT.put(0.96, 150.0);
        flywheelVelocityLUT.put(1.16, 155.0);
        flywheelVelocityLUT.put(1.58, 160.0);
        flywheelVelocityLUT.put(2.07, 165.0);
        flywheelVelocityLUT.put(2.37, 170.0);
        flywheelVelocityLUT.put(2.47, 170.0);
        flywheelVelocityLUT.put(2.70, 170.0);
        flywheelVelocityLUT.put(2.94, 175.0);
        flywheelVelocityLUT.put(3.48, 175.0);
        flywheelVelocityLUT.put(3.92, 180.0);
        flywheelVelocityLUT.put(4.35, 185.0);
        flywheelVelocityLUT.put(4.84, 190.0);

        timeOfFlightLUT.put(5.68, 1.16);
        timeOfFlightLUT.put(4.55, 1.12);
        timeOfFlightLUT.put(3.15, 1.11);
        timeOfFlightLUT.put(1.88, 1.09);
        timeOfFlightLUT.put(1.38, 0.90);

        passingHoodAngleLUT.put(5.46, Rotation2d.fromDegrees(38.0));
        passingHoodAngleLUT.put(6.62, Rotation2d.fromDegrees(38.0));
        passingHoodAngleLUT.put(7.80, Rotation2d.fromDegrees(38.0));
        passingHoodAngleLUT.put(17.16, Rotation2d.fromDegrees(38.0));

        passingFlywheelVelocityLUT.put(5.46, 160.0);
        passingFlywheelVelocityLUT.put(6.62, 180.0);
        passingFlywheelVelocityLUT.put(7.80, 200.0);
        passingFlywheelVelocityLUT.put(17.16, 360.0);

        passingTimeOfFlightLUT.put(5.46, 1.27);
        passingTimeOfFlightLUT.put(6.62, 1.39);
        passingTimeOfFlightLUT.put(7.8, 1.49);
        passingTimeOfFlightLUT.put(11.0, 1.75);
        passingTimeOfFlightLUT.put(13.0, 1.76);
        passingTimeOfFlightLUT.put(17.16, 2.16);

        passingPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Passing/HoodAngle", hoodAngleLUT.get(passingPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Passing/FlywheelSpeed",flywheelVelocityLUT.get(passingPresetDistance))
        );

        hubPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Hub/HoodAngle", hoodAngleLUT.get(hubPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Hub/FlywheelSpeed", flywheelVelocityLUT.get(hubPresetDistance))
        );

        towerPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Tower/HoodAngle", hoodAngleLUT.get(towerPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Tower/FlywheelSpeed", flywheelVelocityLUT.get(towerPresetDistance))
        );

        trenchPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Trench/HoodAngle", hoodAngleLUT.get(trenchPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Trench/FlywheelSpeed", flywheelVelocityLUT.get(trenchPresetDistance))
        );

        outpostPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Outpost/HoodAngle", hoodAngleLUT.get(outpostPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Outpost/FlywheelSpeed", flywheelVelocityLUT.get(outpostPresetDistance))
        );

        hoodMinPreset = new ShootingPreset(
            new LoggedTunableNumber("ShootingCalculator/Presets/HoodMin/HoodAngle", HoodConstants.kHoodMinimumPosition.in(Degrees)),
            new LoggedTunableNumber("ShooterCalculator/Presets/HoodMin/FlywheelSpeed", 50)
        );

        hoodMaxPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/HoodMax/HoodAngle", HoodConstants.kHoodMaximumPosition.in(Degrees)),
            new LoggedTunableNumber("ShooterCalculator/Presets/HoodMax/FlywheelSpeed", 50)
        );
    }

    private static final Bounds towerBound = new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));
    private static final Bounds nearHubBound = new Bounds(
        FieldConstants.LinesVertical.neutralZoneNear,
        FieldConstants.LinesVertical.neutralZoneNear + Units.inchesToMeters(120),
        FieldConstants.LinesHorizontal.rightBumpStart,
        FieldConstants.LinesHorizontal.leftBumpEnd
    );

    private static final Bounds farHubBound = new Bounds(
        FieldConstants.LinesVertical.oppAllianceZone,
        FieldConstants.fieldLength,
        FieldConstants.LinesHorizontal.rightBumpStart,
        FieldConstants.LinesHorizontal.leftBumpEnd
    );

    public static final LoggedTunableNumber passingIdleSpeed = new LoggedTunableNumber("ShooterCalculator/PassingIdleSpeed", 100);

    private static ShooterCalculator instance;
    public static ShooterCalculator getInstance(RobotState robotState) {
        if (instance == null) instance = new ShooterCalculator(robotState);
        return instance;
    }
    
    private ShooterCalculator(RobotState robotState) {
        this.robotState = robotState;
    }

    public static double getMinTimeOfFlight() {
        return timeOfFlightLUT.get(minDistance);
    }

    public static double getMaxTimeOfFlight() {
        return timeOfFlightLUT.get(maxDistance);
    }

    public double getNaiveTOF(double distance) {
        return timeOfFlightLUT.get(distance);
    }

    public void clearLaunchingParameters() {
        latestParameters = null;
    }

    public ShootingParameters getParameters() {
        boolean passing = AllianceFlipUtility.applyX(robotState.getLatestFieldToRobot().getValue().getX()) > FieldConstants.LinesVertical.hubCenter;

        if (latestParameters != null) {
            return latestParameters;
        }

        Pose2d rawPose = robotState.getLatestFieldToRobot().getValue();
        ChassisSpeeds robotVelocity = robotState.getLatestMeasuredRobotRelativeChassisSpeeds();
        
        double dt = phaseDelay;
        double ax = (robotVelocity.vxMetersPerSecond - previousRobotVelocityVx)    / GlobalConstants.kLoopPeriodSeconds;
        double ay = (robotVelocity.vyMetersPerSecond - previousRobotVelocityVy)    / GlobalConstants.kLoopPeriodSeconds;
        double aW = (robotVelocity.omegaRadiansPerSecond - previousRobotVelocityOmega) / GlobalConstants.kLoopPeriodSeconds;

        Pose2d estimatedPose = rawPose.exp(new Twist2d(
            robotVelocity.vxMetersPerSecond * dt + 0.5 * ax * dt * dt,
            robotVelocity.vyMetersPerSecond * dt + 0.5 * ay * dt * dt,
            robotVelocity.omegaRadiansPerSecond * dt + 0.5 * aW * dt * dt
        ));

        previousRobotVelocityVx = robotVelocity.vxMetersPerSecond;
        previousRobotVelocityVy = robotVelocity.vyMetersPerSecond;
        previousRobotVelocityOmega = robotVelocity.omegaRadiansPerSecond;

        Pose2d launcherPose = estimatedPose.transformBy(ShooterConstants.kRobotToShooterTransform.);
 
        // Hub or passing target in field coordinates
        Translation2d target = passing ? getPassingTarget() : AllianceFlipUtility.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        double naiveDistance = target.getDistance(launcherPose.getTranslation());

        ChassisSpeeds fieldSetpoint = RobotState.getInstance().getFieldSetpointVelocity();
        ChassisSpeeds launcherVelocity = DriverStation.isAutonomous() ? fieldSetpoint : addRotationalVelocityComponent(fieldSetpoint, launcherPose);
 
        double vx = launcherVelocity.vxMetersPerSecond;
        double vy = launcherVelocity.vyMetersPerSecond;
        double robotSpeed = Math.hypot(vx, vy);
 
        // Speed cap — above this we are outside LUT calibration range
        if (robotSpeed > solverConfig.maxSOTMSpeed) {
            latestParameters = ShootingParameters.INVALID;
            return latestParameters;
        }

        boolean velocityFiltered = robotSpeed < solverConfig.minSOTMSpeed;

        double solvedTOF;
        double solvedDistance;
        int    iterationsUsed;
 
        if (velocityFiltered) {
            solvedTOF = lookupTOF(passing, naiveDistance);
            solvedDistance = naiveDistance;
            iterationsUsed = 0;
        } else {
            // Warm-start from previous cycle when available (usually converges in 1–2 iterations)
            double tof = previousTOF > 0 ? previousTOF : lookupTOF(passing, naiveDistance);
            solvedDistance = naiveDistance;
            iterationsUsed = 0;
 
            double rx = target.getX() - launcherPose.getX();
            double ry = target.getY() - launcherPose.getY();
 
            for (int i = 0; i < solverConfig.maxIterations; i++) {
                double prevTOF = tof;
 
                double c        = solverConfig.dragCoeff;
                double dragExp  = c < 1e-6 ? 1.0 : Math.exp(-c * tof);
                double driftTOF = c < 1e-6 ? tof : (1.0 - dragExp) / c;
 
                // Projected launcher -> target vector at time t
                double prx  = rx - vx * driftTOF;
                double pry  = ry - vy * driftTOF;
                double dist = Math.hypot(prx, pry);
 
                if (dist < 0.01) {
                    // Degenerate: launcher essentially on top of hub
                    tof = lookupTOF(passing, naiveDistance);
                    iterationsUsed = solverConfig.maxIterations + 1;
                    break;
                }
 
                double lookupT = lookupTOF(passing, dist);
 
                // Derivative: df/dt via chain rule
                double dPrime = -dragExp * (prx * vx + pry * vy) / dist;
                double gPrime = tofDerivative(passing, dist);
                double f = lookupT - tof;
                double fPrime = gPrime * dPrime - 1.0;
 
                tof = Math.abs(fPrime) > 0.01 ? tof - f / fPrime : lookupT; // fixed-point fallback when denominator is near-zero
 
                tof = MathUtil.clamp(tof, solverConfig.tofMin, solverConfig.tofMax);
                iterationsUsed = i + 1;
                solvedDistance = dist;
 
                if (Math.abs(tof - prevTOF) < solverConfig.convergenceTolerance) break;
            }
 
            // Divergence guard
            if (tof > solverConfig.tofMax || tof < 0 || Double.isNaN(tof)) {
                tof = lookupTOF(passing, naiveDistance);
                solvedDistance = naiveDistance;
                iterationsUsed = solverConfig.maxIterations + 1;
            }
 
            solvedTOF = tof;
        }
 
        previousTOF = solvedTOF;
        previousSpeed = robotSpeed;
 
        double effectiveTOF = solvedTOF + solverConfig.mechLatencyMs / 1000.0;
        Rotation2d driveAngle = getDesiredDrivetrainHeadingToShoot(estimatedPose, target, launcherVelocity, solvedTOF);
 
        // Hood and flywheel from LUTs at solved (lookahead) distance
        double hoodAngle = lookupHoodAngle(passing, solvedDistance).getRadians() + Units.degreesToRadians(hoodAngleOffsetDegrees);
        double flywheelVelocity = lookupFlywheel(passing, solvedDistance) + flywheelVelocityOffset;
 
        // Smooth derivatives for feedforward
        if (lastDriveAngle == null) lastDriveAngle = driveAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
 
        double hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / GlobalConstants.kLoopPeriodSeconds);
        double driveVelocity = driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).getRadians() / GlobalConstants.kLoopPeriodSeconds);
 
        lastHoodAngle  = hoodAngle;
        lastDriveAngle = driveAngle;
 
        // "Boxes of bad" — suppress isValid in known bad shooting regions
        Pose2d flippedPose = AllianceFlipUtility.apply(estimatedPose);
        boolean outsideBadBoxes = !(towerBound.contains(flippedPose.getTranslation()) || nearHubBound.contains(flippedPose.getTranslation()) || farHubBound.contains(flippedPose.getTranslation()));
 
        double effectiveMinDist = passing ? passingMinDistance : minDistance;
        double effectiveMaxDist = passing ? passingMaxDistance : maxDistance;
        boolean inRange = solvedDistance >= effectiveMinDist && solvedDistance <= effectiveMaxDist;
        boolean isValid = outsideBadBoxes && inRange;
 
        // Solver quality: converged fast = high quality, diverged = zero
        double solverQuality;
        if (velocityFiltered) {
            solverQuality = 1.0;
        } else if (iterationsUsed > solverConfig.maxIterations) {
            solverQuality = 0.0;
        } else if (iterationsUsed <= 3) {
            solverQuality = 1.0;
        } else {
            solverQuality = MathUtil.interpolate(1.0, 0.1, (double)(iterationsUsed - 3) / (solverConfig.maxIterations - 3));
        }
 
        double headingErrorRad = MathUtil.angleModulus(driveAngle.getRadians() - estimatedPose.getRotation().getRadians());
 
        double confidence = computeConfidence(
            solverQuality, robotSpeed, headingErrorRad,
            solvedDistance, RobotState.getInstance().getVisionConfidence()
        );
 
        latestParameters = new ShootingParameters(
            isValid,
            driveAngle,
            driveVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelVelocity,
            solvedDistance,
            naiveDistance,
            effectiveTOF,
            passing,
            confidence,
            iterationsUsed);
 
        // AdvantageKit logging
        Logger.recordOutput("ShooterCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
        Logger.recordOutput("ShooterCalculator/SolvedDistance", solvedDistance);
        Logger.recordOutput("ShooterCalculator/NaiveDistance", naiveDistance);
        Logger.recordOutput("ShooterCalculator/SolvedTOF", solvedTOF);
        Logger.recordOutput("ShooterCalculator/Confidence", confidence);
        Logger.recordOutput("ShooterCalculator/IterationsUsed", iterationsUsed);
        Logger.recordOutput("ShooterCalculator/FlywheelOffset", flywheelVelocityOffset);
        Logger.recordOutput("ShooterCalculator/IsValid", isValid);
        Logger.recordOutput("ShooterCalculator/Passing", passing);
 
        return latestParameters;
    }

    private static Rotation2d getDesiredDrivetrainHeadingToShoot(Pose2d robotPose, Translation2d target, ChassisSpeeds fieldRelativeVelocity, double timeOfFlight) {
        Translation2d virtualTarget = target.minus(new Translation2d(
            fieldRelativeVelocity.vxMetersPerSecond * timeOfFlight,
            fieldRelativeVelocity.vyMetersPerSecond * timeOfFlight)
        );

        Rotation2d fieldToHubAngle = virtualTarget.minus(robotPose.getTranslation()).getAngle();
        double distanceToVirtualTarget = virtualTarget.getDistance(robotPose.getTranslation());

        Rotation2d hubAngle = new Rotation2d(Math.asin(MathUtil.clamp(ShooterConstants.kRobotToShooterTransform.getTranslation().getY() / distanceToVirtualTarget, -1.0, 1.0)));
        return fieldToHubAngle.plus(hubAngle).plus(ShooterConstants.kRobotToShooterTransform.getRotation().toRotation2d());
    }

    public Translation2d getPassingTarget() {
        double flippedY = AllianceFlipUtility.apply(robotState.getLatestFieldToRobot().getValue()).getY();
        boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

        Translation2d flippedGoalTranslation = AllianceFlipUtility.apply(
            new Translation2d(xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget)
        );

        return flippedGoalTranslation;
    }

    public void incrementHoodAngleOffset(double incrementDegrees) {
        hoodAngleOffsetDegrees += incrementDegrees;
    }
        */
}