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
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;

import lombok.Getter;

public class ShooterCalculator {
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
    private static final double maxFlywheelVelocityOffset = 200.0;

    private static final double dh = 0.01;

    private double lastHoodAngle;
    private Rotation2d lastDriveAngle;

    private ShootingParameters latestParameters = null;

    private static final double minDistance;
    private static final double maxDistance;
    private static final double passingMinDistance;
    private static final double passingMaxDistance;
    private static final double phaseDelaySeconds;

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

    private static final double xPassTarget = Units.inchesToMeters(37); //TODO: Convert to an actual Translation in constants
    private static final double yPassTarget = Units.inchesToMeters(65); //TODO: Convert to an actual Translation in constants

    static {
        minDistance = 0.9;
        maxDistance = 4.9;
        passingMinDistance = 5.4;
        passingMaxDistance = 17.16;
        phaseDelaySeconds = 0.03;

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
            new LoggedTunableNumber("ShooterCalculator/Presets/Passing/FlywheelVelocity",flywheelVelocityLUT.get(passingPresetDistance)),
            new LoggedTunableNumber("ShooterCalculator/Presets/Passing/FlywheelVoltage",0.0)
        );

        hubPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Hub/HoodAngle", hoodAngleLUT.get(hubPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Hub/FlywheelSpeed", flywheelVelocityLUT.get(hubPresetDistance)),
            new LoggedTunableNumber("ShooterCalculator/Presets/Hub/FlywheelVoltage",0.0)
        );

        towerPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Tower/HoodAngle", hoodAngleLUT.get(towerPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Tower/FlywheelSpeed", flywheelVelocityLUT.get(towerPresetDistance)),
            new LoggedTunableNumber("ShooterCalculator/Presets/Tower/FlywheelVoltage",0.0)
        );

        trenchPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Trench/HoodAngle", hoodAngleLUT.get(trenchPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Trench/FlywheelSpeed", flywheelVelocityLUT.get(trenchPresetDistance)),
            new LoggedTunableNumber("ShooterCalculator/Presets/Trench/FlywheelVoltage",0.0)
        );

        outpostPreset = new ShootingPreset(
            new LoggedTunableNumber("ShooterCalculator/Presets/Outpost/HoodAngle", hoodAngleLUT.get(outpostPresetDistance).getDegrees()),
            new LoggedTunableNumber("ShooterCalculator/Presets/Outpost/FlywheelSpeed", flywheelVelocityLUT.get(outpostPresetDistance)),
            new LoggedTunableNumber("ShooterCalculator/Presets/Outpost/FlywheelVoltage",0.0)
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

    private final SolverConfiguration solverConfig = new SolverConfiguration();
 
    // Newton solver warm-start state
    private double previousTOF = -1.0;
    private double previousSpeed = 0.0;
 
    // Previous-cycle robot velocities for second-order pose prediction
    private double previousRobotVelocityVx = 0.0;
    private double previousRobotVelocityVy = 0.0;
    private double previousRobotVelocityOmega = 0.0;
 
    private static ShooterCalculator instance;
 
    public static ShooterCalculator getInstance(RobotState robotState) {
        if (instance == null) instance = new ShooterCalculator(robotState);
        return instance;
    }
 
    /** Access the already-initialised singleton. Throws if called before getInstance(RobotState). */
    public static ShooterCalculator getInstance() {
        if (instance == null) {
            throw new IllegalStateException("ShooterCalculator not yet initialised — call getInstance(RobotState) first");
        }
        return instance;
    }
 
    private ShooterCalculator(RobotState robotState) {
        this.robotState = robotState;
    }
 
    /** Invalidate the per-cycle cache. Call once at the top of Superstructure.periodic(). */
    public void clearLaunchingParameters() {
        latestParameters = null;
    }
 
    public static double getMinTimeOfFlight() { return timeOfFlightLUT.get(minDistance); }
    public static double getMaxTimeOfFlight() { return timeOfFlightLUT.get(maxDistance); }
    public double getNaiveTOF(double distance) { return timeOfFlightLUT.get(distance); }

    public ShootingParameters getParameters() {
        if (latestParameters != null) return latestParameters;
 
        // Profile selection: robot past hub centre line → passing
        boolean passing = AllianceFlipUtility.applyX(robotState.getLatestFieldToRobot().getValue().getX()) > FieldConstants.LinesVertical.hubCenter;
 
        // Second-order latency-compensated pose prediction
        // v*dt + 0.5*a*dt^2 tracks curvature through turns better than first-order
        Pose2d rawPose = robotState.getLatestFieldToRobot().getValue();
        ChassisSpeeds velocity = robotState.getLatestMeasuredRobotRelativeChassisSpeeds();
 
        double dt = phaseDelaySeconds;
        double ax = (velocity.vxMetersPerSecond - previousRobotVelocityVx) / GlobalConstants.kLoopPeriodSeconds;
        double ay = (velocity.vyMetersPerSecond - previousRobotVelocityVy) / GlobalConstants.kLoopPeriodSeconds;
        double aW = (velocity.omegaRadiansPerSecond - previousRobotVelocityOmega) / GlobalConstants.kLoopPeriodSeconds;
 
        Pose2d estimatedPose = rawPose.exp(new Twist2d(
            velocity.vxMetersPerSecond     * dt + 0.5 * ax * dt * dt,
            velocity.vyMetersPerSecond     * dt + 0.5 * ay * dt * dt,
            velocity.omegaRadiansPerSecond * dt + 0.5 * aW * dt * dt)
        );
 
        previousRobotVelocityVx = velocity.vxMetersPerSecond;
        previousRobotVelocityVy = velocity.vyMetersPerSecond;
        previousRobotVelocityOmega = velocity.omegaRadiansPerSecond;
 
        // Launcher position in field coordinates
        Pose2d launcherPose = estimatedPose.transformBy(ShooterConstants.kRobotToShooterTransform);
 
        // Hub or passing target
        Translation2d target = passing ? getPassingTarget(): AllianceFlipUtility.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        double naiveDistance = target.getDistance(launcherPose.getTranslation());
 
        // Field-relative launcher velocity including rotational component (ω×r)
        // Use desired speeds so SOTM leads intended motion, not measured lag
        ChassisSpeeds fieldSpeeds = DriverStation.isAutonomous() ? robotState.getLatestMeasuredFieldRelativeChassisSpeeds() : robotState.getLatestDesiredFieldRelativeChassisSpeeds();
        ChassisSpeeds launcherVelocity = addRotationalVelocityComponent(fieldSpeeds, launcherPose);
 
        double vx = launcherVelocity.vxMetersPerSecond;
        double vy = launcherVelocity.vyMetersPerSecond;
        double robotSpeed = Math.hypot(vx, vy);
 
        if (robotSpeed > solverConfig.maxSOTMSpeed) {
            latestParameters = ShootingParameters.INVALID;
            return latestParameters;
        }
 
        boolean velocityFiltered = robotSpeed < solverConfig.minSOTMSpeed;
 
        // Newton-method SOTM solver
        double solvedTOF;
        double solvedDistance;
        int iterationsUsed;
 
        if (velocityFiltered) {
            solvedTOF = lookupTOF(passing, naiveDistance);
            solvedDistance = naiveDistance;
            iterationsUsed = 0;
        } else {
            double tof = previousTOF > 0 ? previousTOF : lookupTOF(passing, naiveDistance);
 
            solvedDistance = naiveDistance;
            iterationsUsed = 0;
 
            double rx = target.getX() - launcherPose.getX();
            double ry = target.getY() - launcherPose.getY();
 
            for (int i = 0; i < solverConfig.maxIterations; i++) {
                double prevTOF = tof;
 
                double c = solverConfig.dragCoeff;
                double dragExp = c < 1e-6 ? 1.0 : Math.exp(-c * tof);
                double driftTOF = c < 1e-6 ? tof : (1.0 - dragExp) / c;
 
                double prx = rx - vx * driftTOF;
                double pry = ry - vy * driftTOF;
                double dist = Math.hypot(prx, pry);
 
                if (dist < 0.01) {
                    tof = lookupTOF(passing, naiveDistance);
                    iterationsUsed = solverConfig.maxIterations + 1;
                    break;
                }
 
                double lookupT = lookupTOF(passing, dist);
                double dPrime = -dragExp * (prx * vx + pry * vy) / dist;
                double gPrime = tofDerivative(passing, dist);
                double f = lookupT - tof;
                double fPrime = gPrime * dPrime - 1.0;
 
                tof = Math.abs(fPrime) > 0.01 ? tof - f / fPrime : lookupT;
 
                tof = MathUtil.clamp(tof, solverConfig.tofMin, solverConfig.tofMax);
                iterationsUsed = i + 1;
                solvedDistance = dist;
 
                if (Math.abs(tof - prevTOF) < solverConfig.convergenceTolerance) break;
            }
 
            if (tof > solverConfig.tofMax || tof < 0.0 || Double.isNaN(tof)) {
                tof = lookupTOF(passing, naiveDistance);
                solvedDistance = naiveDistance;
                iterationsUsed = solverConfig.maxIterations + 1;
            }
 
            solvedTOF = tof;
        }
 
        previousTOF = solvedTOF;
        previousSpeed = robotSpeed;
 
        double effectiveTOF = solvedTOF + solverConfig.mechLatencyMs / 1000.0;
 
        // Drivetrain heading — accounts for launcher offset and SOTM velocity compensation
        Rotation2d driveAngle = getDesiredDrivetrainHeadingToShoot(estimatedPose, target, launcherVelocity, solvedTOF);
 
        // Hood and flywheel from LUTs
        double hoodAngle = lookupHoodAngle(passing, solvedDistance).getRadians() + Units.degreesToRadians(hoodAngleOffsetDegrees);
        double flywheelVelocity = lookupFlywheel(passing, solvedDistance) + flywheelVelocityOffset;
 
        // Smooth derivatives for feedforward
        if (lastDriveAngle == null) lastDriveAngle = driveAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
 
        double hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / GlobalConstants.kLoopPeriodSeconds);
        double driveVelocity = driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).getRadians() / GlobalConstants.kLoopPeriodSeconds);
 
        lastHoodAngle  = hoodAngle;
        lastDriveAngle = driveAngle;
 
        // Validity
        Pose2d flippedPose = AllianceFlipUtility.apply(estimatedPose);
        boolean outsideBadBoxes = !(towerBound.contains(flippedPose.getTranslation()) || nearHubBound.contains(flippedPose.getTranslation()) || farHubBound.contains(flippedPose.getTranslation()));
 
        double  effectiveMinDist = passing ? passingMinDistance : minDistance;
        double  effectiveMaxDist = passing ? passingMaxDistance : maxDistance;
        boolean inRange = solvedDistance >= effectiveMinDist && solvedDistance <= effectiveMaxDist;
        boolean isValid = outsideBadBoxes && inRange;
 
        // Confidence
        double solverQuality;
        if (velocityFiltered) {
            solverQuality = 1.0;
        } else if (iterationsUsed > solverConfig.maxIterations) {
            solverQuality = 0.0;
        } else if (iterationsUsed <= 3) {
            solverQuality = 1.0;
        } else {
            solverQuality = MathUtil.interpolate(1.0, 0.1,(double) (iterationsUsed - 3) / (solverConfig.maxIterations - 3));
        }
 
        double headingErrorRad = MathUtil.angleModulus(driveAngle.getRadians() - estimatedPose.getRotation().getRadians());
        double confidence = computeConfidence(solverQuality, robotSpeed, headingErrorRad, solvedDistance, robotState.getVisionConfidence());
 
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
            iterationsUsed
        );
 
        Logger.recordOutput("ShooterCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
        Logger.recordOutput("ShooterCalculator/LauncherPose", launcherPose);
        Logger.recordOutput("ShooterCalculator/SolvedDistance", solvedDistance);
        Logger.recordOutput("ShooterCalculator/NaiveDistance", naiveDistance);
        Logger.recordOutput("ShooterCalculator/SolvedTOF", solvedTOF);
        Logger.recordOutput("ShooterCalculator/EffectiveTOF", effectiveTOF);
        Logger.recordOutput("ShooterCalculator/Confidence", confidence);
        Logger.recordOutput("ShooterCalculator/IterationsUsed", iterationsUsed);
        Logger.recordOutput("ShooterCalculator/FlywheelOffset", flywheelVelocityOffset);
        Logger.recordOutput("ShooterCalculator/HoodOffsetDegrees", hoodAngleOffsetDegrees);
        Logger.recordOutput("ShooterCalculator/IsValid", isValid);
        Logger.recordOutput("ShooterCalculator/Passing", passing);
        Logger.recordOutput("ShooterCalculator/DriveAngle", driveAngle);
        Logger.recordOutput("ShooterCalculator/RobotSpeed", robotSpeed);
        Logger.recordOutput("ShooterCalculator/VelocityFiltered", velocityFiltered);
 
        return latestParameters;
    }
 
    /**
     * Returns the robot heading required so the LAUNCHER (not the robot centre)
     * points at the velocity-compensated virtual target.
     *
     * The virtual target is the real hub shifted backward by robot velocity × TOF.
     * Aiming there means the fuel arrives at the real hub despite robot motion.
     *
     * DrivetrainFactory.autoAim() should use getParameters().driveAngle() when SOTM
     * is active and confidence is sufficient, falling back to this method with
     * timeOfFlight = 0 for purely geometric (stationary) aiming.
     *
     * @param robotPose Current (latency-compensated) robot pose
     * @param target Field-relative target translation
     * @param fieldRelativeVelocity  Field-relative launcher velocity
     * @param timeOfFlight Solved TOF in seconds (pass 0 for stationary aim)
     */
    public static Rotation2d getDesiredDrivetrainHeadingToShoot(Pose2d robotPose, Translation2d target, ChassisSpeeds fieldRelativeVelocity, double timeOfFlight) {
        // Shift hub backward by velocity×TOF to get the virtual aim point
        Translation2d virtualTarget = target.minus(new Translation2d(fieldRelativeVelocity.vxMetersPerSecond * timeOfFlight,fieldRelativeVelocity.vyMetersPerSecond * timeOfFlight));
        Rotation2d fieldToHubAngle = virtualTarget.minus(robotPose.getTranslation()).getAngle();
 
        double distanceToVirtualTarget = virtualTarget.getDistance(robotPose.getTranslation());
 
        // asin(opposite/hypotenuse) — lateral launcher offset correction
        // Corrects for the launcher not being at the robot centre
        double lateralOffset = ShooterConstants.kRobotToShooterTransform.getTranslation().getY();
        Rotation2d lateralCorrection = new Rotation2d(Math.asin(MathUtil.clamp(lateralOffset / distanceToVirtualTarget, 1.0, 1.0)));
 
        return fieldToHubAngle.plus(lateralCorrection).plus(ShooterConstants.kRobotToShooterTransform.getRotation());
    }
 
    public Translation2d getPassingTarget() {
        double  flippedY = AllianceFlipUtility.apply(robotState.getLatestFieldToRobot().getValue()).getY();
        boolean mirror   = flippedY > FieldConstants.LinesHorizontal.center;
        return AllianceFlipUtility.apply(new Translation2d(xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget));
    }
 
    /** Nudge flywheel speed ±delta. Clamped to ±200. Bind to copilot D-pad. */
    public void adjustFlywheelOffset(double delta) {
        flywheelVelocityOffset = MathUtil.clamp(flywheelVelocityOffset + delta, -maxFlywheelVelocityOffset, maxFlywheelVelocityOffset);
    }
 
    public void resetFlywheelOffset() {
        flywheelVelocityOffset  = 0.0; 
    }

    public double getFlywheelOffset() {
        return flywheelVelocityOffset; 
    }

    public void incrementHoodAngleOffset(double deg) {
        hoodAngleOffsetDegrees += deg; 
    }

    public void resetHoodAngleOffset() { 
        hoodAngleOffsetDegrees   = 0.0; 
    }
 
    /**
     * Reset warm-start state. Call after a pose reset so stale TOF
     * doesn't seed the solver on the next cycle.
     */
    public void resetWarmStart() {
        previousTOF = -1.0;
        previousSpeed = 0.0;
        previousRobotVelocityVx = 0.0;
        previousRobotVelocityVy = 0.0;
        previousRobotVelocityOmega = 0.0;
    }
 
    private static double lookupTOF(boolean passing, double distance) {
        return passing ? passingTimeOfFlightLUT.get(distance) : timeOfFlightLUT.get(distance);
    }
 
    private static Rotation2d lookupHoodAngle(boolean passing, double distance) {
        return passing ? passingHoodAngleLUT.get(distance) : hoodAngleLUT.get(distance);
    }
 
    private static double lookupFlywheel(boolean passing, double distance) {
        return passing ? passingFlywheelVelocityLUT.get(distance) : flywheelVelocityLUT.get(distance);
    }
 
    private static double tofDerivative(boolean passing, double d) {
        return (lookupTOF(passing, d + dh) - lookupTOF(passing, d - dh)) / (2.0 * dh);
    }
 
    private static ChassisSpeeds addRotationalVelocityComponent(ChassisSpeeds fieldSpeeds, Pose2d launcherPose) {
        double heading = launcherPose.getRotation().getRadians();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double offX = ShooterConstants.kRobotToShooterTransform.getTranslation().getX();
        double offY = ShooterConstants.kRobotToShooterTransform.getTranslation().getY();
        double fieldOffX =  offX * cosH - offY * sinH;
        double fieldOffY =  offX * sinH + offY * cosH;
        double omega = fieldSpeeds.omegaRadiansPerSecond;

        return new ChassisSpeeds(fieldSpeeds.vxMetersPerSecond + (-fieldOffY) * omega, fieldSpeeds.vyMetersPerSecond + ( fieldOffX) * omega, omega);
    }

    private double computeConfidence(double solverQuality, double robotSpeed, double headingErrorRadians, double distance, double visionConfidence) {
        double convergenceQuality = solverQuality;
        double speedDelta = Math.abs(robotSpeed - previousSpeed);
        double velocityStability = MathUtil.clamp(1.0 - speedDelta / 0.5, 0.0, 1.0);
        double visionConf = MathUtil.clamp(visionConfidence, 0.0, 1.0);
        double distanceScale = MathUtil.clamp(solverConfig.headingReferenceDistance / distance, 0.5, 2.0);
        double speedScale = 1.0 / (1.0 + solverConfig.headingSpeedScalar * robotSpeed);
        double scaledMaxErr = solverConfig.headingMaxErrorRadians * distanceScale * speedScale;
        double headingAcc = MathUtil.clamp(1.0 - Math.abs(headingErrorRadians) / scaledMaxErr, 0.0, 1.0);
 
        double rangeSpan = maxDistance - minDistance;
        double rangeFrac = (distance - minDistance) / rangeSpan;
        double distInRange = MathUtil.clamp(1.0 - 2.0 * Math.abs(rangeFrac - 0.5), 0.0, 1.0);
 
        double[] components = { convergenceQuality, velocityStability, visionConf, headingAcc, distInRange };
        double[] weights = { solverConfig.wConvergence, solverConfig.wVelocityStability, solverConfig.wVisionConfidence, solverConfig.wHeadingAccuracy, solverConfig.wDistanceInRange };
 
        double logSum = 0.0, sumW = 0.0;
        for (int i = 0; i < components.length; i++) {
            if (components[i] <= 0.0) return 0.0;
            logSum += weights[i] * Math.log(components[i]);
            sumW   += weights[i];
        }
 
        return sumW > 0 ? MathUtil.clamp(Math.exp(logSum / sumW) * 100.0, 0.0, 100.0) : 0.0;
    }

    public static class SolverConfiguration {
        // Newton solver
        public int maxIterations = 20;
        public double convergenceTolerance = 0.001; // seconds
        public double tofMin = 0.05;
        public double tofMax = 5.0;
 
        // Below this field speed (m/s) skip SOTM and aim straight
        public double minSOTMSpeed = 0.10;
        // Above this field speed (m/s) refuse to shoot — outside calibration
        public double maxSOTMSpeed = 3.0;
 
        // Drag coefficient (1/s). displacement = v*(1-e^(-c*t))/c
        // Set to 0 to disable drag compensation.
        public double dragCoeff = 0.24;
 
        // Latency budgets
        public double phaseDelayMs = 30.0; // vision/command pipeline lag (ms)
        public double mechLatencyMs = 20.0; // mechanism response lag (ms)
 
        // Confidence weights (weighted geometric mean — one zero kills total)
        public double wConvergence = 1.0;
        public double wVelocityStability = 0.8;
        public double wVisionConfidence = 1.2;
        public double wHeadingAccuracy = 1.5;
        public double wDistanceInRange = 0.5;
 
        // Heading accuracy thresholds
        public double headingMaxErrorRadians = Math.toRadians(3.0);
        public double headingSpeedScalar = 1.0;
        public double headingReferenceDistance = 2.5; // meters
    }
}