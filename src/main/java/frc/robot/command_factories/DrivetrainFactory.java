package frc.robot.command_factories;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

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
