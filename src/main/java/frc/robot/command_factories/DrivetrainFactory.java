package frc.robot.command_factories;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotState;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DrivetrainFactory {
    private static final PIDController driveToPointController = new PIDController(3.6, 0, 0.3);
    private static final PIDController rotationController = new PIDController(3, 0, 0.1);

    private static final SwerveRequest.FieldCentricFacingAngle facingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withHeadingPID(5.1, 0, 0.0);

    public static Command handleTeleopDrive(Drivetrain drivetrain, RobotState robotState, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier isFieldCentric) {
        return Commands.run(() -> {
            ChassisSpeeds speeds = calculateSpeedsBasedOnJoystickInputs(drivetrain, robotState, throttleSupplier, strafeSupplier, rotationSupplier);

            if(isFieldCentric.getAsBoolean()) {
                drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                );
            } else {
                drivetrain.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                );
            }
        }, drivetrain);
    }

    public static Command driveToPoint(Drivetrain drivetrain, RobotState robotState, Supplier<Pose2d> targetPose, double constraintedMaximumLinearVelocity, double constraintedMaximumAngularVelocity) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        return Commands.runOnce(() -> {
            driveToPointController.reset();
            rotationController.reset();
        }).andThen(Commands.run(() -> {
            Translation2d translationToDesiredPoint = targetPose.get().getTranslation().minus(robotState.getLatestFieldToRobot().getValue().getTranslation());
            double linearDistance = translationToDesiredPoint.getNorm();
            double frictionConstant = 0.0;

            if (linearDistance >= Units.inchesToMeters(0.5)) {
                frictionConstant = 0.02 * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);
            }

            Rotation2d directionOfTravel = translationToDesiredPoint.getAngle();
            double velocityOutput = 0.0;

            double currentHeading = robotState.getLatestFieldToRobot().getValue().getRotation().getRadians();
            double targetHeading = targetPose.get().getRotation().getRadians();

            double angularVelocity = rotationController.calculate(currentHeading, targetHeading);

            velocityOutput = Math.min(
                Math.abs(driveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                constraintedMaximumLinearVelocity
            );

            double xComponent = velocityOutput * directionOfTravel.getCos();
            double yComponent = velocityOutput * directionOfTravel.getSin();

            Logger.recordOutput("Drivetrain/DriveToPoint/VelocitySetpointX", xComponent);
            Logger.recordOutput("Drivetrain/DriveToPoint/VelocitySetpointY", yComponent);
            Logger.recordOutput("Drivetrain/DriveToPoint/VelocityOutput", velocityOutput);
            Logger.recordOutput("Drivetrain/DriveToPoint/LinearDistance", linearDistance);
            Logger.recordOutput("Drivetrain/DriveToPoint/DirectionOfTravel", directionOfTravel);
            Logger.recordOutput("Drivetrain/DriveToPoint/DesiredPoint", targetPose.get());
            Logger.recordOutput("Drivetrain/DriveToPoint/DesiredHeading", targetHeading);
            Logger.recordOutput("Drivetrain/DriveToPoint/CurrentHeading", currentHeading);

            angularVelocity = Double.isNaN(constraintedMaximumAngularVelocity) 
                ? angularVelocity 
                :  MathUtil.clamp(angularVelocity, -constraintedMaximumAngularVelocity, constraintedMaximumAngularVelocity);

            drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(xComponent)
                .withVelocityY(yComponent)
                .withRotationalRate(angularVelocity)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            );

        }, drivetrain)).until(() ->  {
            double error = targetPose.get().getTranslation().getDistance(robotState.getLatestFieldToRobot().getValue().getTranslation());
            return MathUtil.isNear(0.0, error, Units.inchesToMeters(1));
        }).withName("StationaryDriveToPoint");
    }

    public static Command autoAim(Drivetrain drivetrain, RobotState robotState, Supplier<Translation2d> targetTranslation, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier) {
        return Commands.run(() -> {
            Pose2d robotPose = robotState.getLatestFieldToRobot().getValue();
            Rotation2d desiredHeading = getShooterAimedHeading(
                robotPose,
                targetTranslation.get(),
                ShooterConstants.kRobotToShooterTransform
            );

            drivetrain.setControl(facingAngleRequest
                .withVelocityX(throttleSupplier.getAsDouble())
                .withVelocityY(strafeSupplier.getAsDouble())
                .withTargetDirection(desiredHeading)
            );

            Logger.recordOutput("Drivetrain/AutoAim/DesiredHeading", desiredHeading);
            Logger.recordOutput("Drivetrain/AutoAim/CurrentHeading", robotPose.getRotation());
            Logger.recordOutput("Drivetrain/AutoAim/HeadingErrorDegrees", desiredHeading.minus(robotPose.getRotation()).getDegrees());
            Logger.recordOutput("Drivetrain/AutoAim/TargetTranslation", new Pose2d(targetTranslation.get(), Rotation2d.kZero));
            Logger.recordOutput("Drivetrain/AutoAim/IsAimed", isAimed(robotState, targetTranslation.get()));

        }, drivetrain)
            .until(() -> isAimed(robotState, targetTranslation.get()))
            .andThen(Commands.runOnce(drivetrain::holdXStance))
            .withName("Auto Aim While Driving");
    }

    private static Rotation2d getShooterAimedHeading(Pose2d robotPose, Translation2d target, Transform2d launcherTransform) {
        Rotation2d fieldToTargetAngle = target.minus(robotPose.getTranslation()).getAngle();

        double distanceToTarget = target.getDistance(robotPose.getTranslation());
        double lateralOffset = launcherTransform.getTranslation().getY();

        Rotation2d lateralCorrection = new Rotation2d(Math.asin(MathUtil.clamp(lateralOffset / distanceToTarget, -1.0, 1.0)));
        return fieldToTargetAngle.plus(lateralCorrection).plus(launcherTransform.getRotation()).plus(Rotation2d.fromDegrees(5));
    }

    public static boolean isAimed(RobotState robotState, Translation2d target) {
        Pose2d robotPose = robotState.getLatestFieldToRobot().getValue();
        Rotation2d desiredHeading = getShooterAimedHeading(
            robotPose,
            target,
            ShooterConstants.kRobotToShooterTransform
        );

        return Math.abs(robotPose.getRotation().minus(desiredHeading).getDegrees()) <= Degrees.of(2.0).baseUnitMagnitude();
    }

    private static ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(Drivetrain drivetrain, RobotState robotState, DoubleSupplier throttleSuppler, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        if(DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double magnitudeX = MathUtil.applyDeadband(throttleSuppler.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeY = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeTheta = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), ControllerConstants.kControllerDeadband);

        magnitudeTheta = Math.copySign(magnitudeTheta * magnitudeTheta, magnitudeTheta);

        double velocityX = magnitudeX * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);
        double velocityY = magnitudeY * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);
        double velocityTheta = magnitudeTheta * DrivetrainConstants.kMaximumRotationalVelocity.in(RadiansPerSecond);

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(robotState.getLatestMeasuredRobotRelativeChassisSpeeds().omegaRadiansPerSecond * -0.03); // TODO: Shew Compenstation to go in DrivetrainConstnats

        return ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(velocityX, velocityY, velocityTheta), robotState.getLatestFieldToRobot().getValue().getRotation()), 
            robotState.getLatestFieldToRobot().getValue().getRotation().plus(skewCompensationFactor)
        );
    }
}
