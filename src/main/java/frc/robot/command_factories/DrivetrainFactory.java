package frc.robot.command_factories;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import org.dyn4j.geometry.Rotation;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotState;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DrivetrainFactory {
    public static Command handleTeleopDrive(Drivetrain drivetrain, RobotState robotState, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, boolean isFieldCentric) {
        return Commands.run(() -> {
            ChassisSpeeds speeds = calculateSpeedsBasedOnJoystickInputs(drivetrain, robotState, throttleSupplier, strafeSupplier, rotationSupplier);

            if(isFieldCentric) {
                drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                );
            } else {
                drivetrain.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                );
            }
        }, drivetrain);
    }

    /*
    
    public static Command aimAtHub(Drivetrain drivetrain, RobotState robotState, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier) {
        SwerveRequest.FieldCentricFacingAngle aimRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond) * 0.05)  // 5% deadband
            .withRotationalDeadband(DrivetrainConstants.kMaximumRotationalVelocity.in(RadiansPerSecond) * 0.05);
        
        return Commands.run(() -> {
            double magnitudeX = MathUtil.applyDeadband(throttleSupplier.getAsDouble(),  ControllerConstants.kControllerDeadband);
            double magnitudeY = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), ControllerConstants.kControllerDeadband);
            
            double velocityX = robotState.isRedAlliance()  ? -magnitudeX * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond) : magnitudeX * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);
            double velocityY = robotState.isRedAlliance() ? -magnitudeY * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond) : magnitudeY * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);
            
            Rotation2d targetAngle = Rotation2d.fromRadians(robotState.getAngleToTarget());
            
            // Apply skew compensation for translation
            Rotation2d skewCompensationFactor = Rotation2d.fromRadians(robotState.getLatestMeasuredRobotRelativeChassisSpeeds().omegaRadiansPerSecond * -0.03);
            drivetrain.setControl(aimRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withTargetDirection(targetAngle.plus(skewCompensationFactor))
            );
        }, drivetrain).withName("Auto Align Hub UNLOCKED");
    }
   
    public static Command aimAtHubLocked(Drivetrain drivetrain, RobotState robotState) {
        SwerveRequest.FieldCentricFacingAngle aimRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond) * 0.05)
            .withRotationalDeadband(DrivetrainConstants.kMaximumRotationalVelocity.in(RadiansPerSecond) * 0.05);
        
        return Commands.run(() -> {
            Rotation2d targetAngle = Rotation2d.fromRadians(robotState.getAngleToTarget());
            
            // No translation - only rotation
            drivetrain.setControl(aimRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withTargetDirection(targetAngle)
            );
        }, drivetrain).withName("Auto Align Hub LOCKED");
    }

    */

    private static ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(Drivetrain drivetrain, RobotState robotState, DoubleSupplier throttleSuppler, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        if(DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double magnitudeX = MathUtil.applyDeadband(throttleSuppler.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeY = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), ControllerConstants.kControllerDeadband);
        double magnitudeTheta = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), ControllerConstants.kControllerDeadband);

        magnitudeTheta = Math.copySign(magnitudeTheta * magnitudeTheta, magnitudeTheta);

        double velocityX = robotState.isRedAlliance() ? -magnitudeX * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond) : magnitudeX * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);
        double velocityY = robotState.isRedAlliance() ? -magnitudeY * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond) : magnitudeY * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);
        double velocityTheta = robotState.isRedAlliance() ? -magnitudeTheta * DrivetrainConstants.kMaximumRotationalVelocity.in(RadiansPerSecond) : magnitudeTheta * DrivetrainConstants.kMaximumRotationalVelocity.in(RadiansPerSecond);

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(robotState.getLatestMeasuredRobotRelativeChassisSpeeds().omegaRadiansPerSecond * -0.03); // TODO: Shew Compenstation to go in DrivetrainConstnats

        return ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(velocityX, velocityY, velocityTheta), robotState.getLatestFieldToRobot().getValue().getRotation()), 
            robotState.getLatestFieldToRobot().getValue().getRotation().plus(skewCompensationFactor)
        );
    }
}
