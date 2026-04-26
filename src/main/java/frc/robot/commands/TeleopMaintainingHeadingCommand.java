package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.Util;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.minolib.math.EqualsUtility;
import frc.minolib.swerve.SwerveModuleType;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TeleopMaintainingHeadingCommand extends Command {
    /*
    private final RobotState robotState;
    private final RobotContainer robotContainer;
    protected Drivetrain drivetrain;
    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private Optional<Rotation2d> headingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;

    private final SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
        .withDeadband(DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond) * ControllerConstants.kControllerDeadband) // Add a 5% deadband in open loop
        .withRotationalDeadband(DrivetrainConstants.kMaximumRotationalVelocity.in(RadiansPerSecond) * ControllerConstants.kControllerDeadband)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond) * 0.05)
        .withDriveRequestType(DriveRequestType.Velocity);
                    
    public TeleopMaintainingHeadingCommand(Drivetrain drivetrain, RobotState robotState, RobotContainer robotContainer, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrain;
        this.robotState = robotState;
        this.robotContainer = robotContainer;
        this.throttleSupplier = throttleSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;

        driveWithHeading.HeadingController.setPID(
            5.0,
            0.0,
            0.0
        );

        addRequirements(drivetrain);
        setName("Swerve Drive Maintain Heading");

        if (Robot.isSimulation()) {
            driveNoHeading.DriveRequestType = DriveRequestType.OpenLoopVoltage;
            driveWithHeading.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        }
    }

    @Override
    public void initialize() {
        headingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = throttleSupplier.getAsDouble() * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);
        double strafe = strafeSupplier.getAsDouble() * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond);

        double turnFieldFrame = rotationSupplier.getAsDouble();
        double throttleFieldFrame = robotState.isRedAlliance() ? -throttle : throttle;
        double strafeFieldFrame = robotState.isRedAlliance() ? -strafe : strafe;

        if (Math.abs(turnFieldFrame) > ControllerConstants.kControllerDeadband) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }
        
        if (Math.abs(turnFieldFrame) > ControllerConstants.kControllerDeadband || (EqualsUtility.epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25) && Math.abs(robotState.getLatestMeasuredRobotRelativeChassisSpeeds().omegaRadiansPerSecond) > Math.toRadians(10))) {
            drivetrain.setControl(driveNoHeading
                .withVelocityX(throttleFieldFrame)
                .withVelocityY(strafeFieldFrame)
                .withRotationalRate(turnFieldFrame * DrivetrainConstants.kMaximumRotationalVelocity.in(RadiansPerSecond))
            );

            headingSetpoint = Optional.empty();
            Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
        } else {
            if (headingSetpoint.isEmpty()) {
                headingSetpoint = Optional.of(robotState.getLatestFieldToRobot().getValue().getRotation());
            }

            Logger.recordOutput("DriveMaintainHeading/throttleFieldFrame", throttleFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/strafeFieldFrame", strafeFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/mHeadingSetpoint", headingSetpoint.get());

            if (mRobotContainer.getCoralStateTracker().getCurrentPosition() != CoralPosition.NONE
                    && mRobotContainer.getModalControls().coralMode().getAsBoolean()
                    && mRobotContainer
                            .getModalSuperstructureTriggers()
                            .getEnableSnapHeadingCoral()
                            .get()) {
                double targetAngle =
                        mRobotContainer
                                .getStateMachine()
                                .getReefFaceAngleRadians(
                                        mRobotContainer
                                                .getStateMachine()
                                                .getClosestFace(mRobotState.isRedAlliance()));
                mDrivetrain.setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(
                                        mRobotState.isRedAlliance()
                                                ? Util.flipRedBlue(new Rotation2d(targetAngle))
                                                : new Rotation2d(targetAngle)));
                Logger.recordOutput("DriveMaintainHeading/reefHeadingLock", targetAngle);
                // update heading setpoint to avoid snapping back to previous setpoint after scoring
                mHeadingSetpoint =
                        Optional.of(mRobotState.getLatestFieldToRobot().getValue().getRotation());

                // barge heading lock
            } else if (mRobotContainer.getModalControls().algaeClimbMode().getAsBoolean()
                    && mRobotContainer
                                    .getModalSuperstructureTriggers()
                                    .getLatestAlgaeStageState()
                                    .get()
                            == SuperstructureState.STAGE_BARGE
                    && mRobotContainer
                            .getModalSuperstructureTriggers()
                            .getEnableSnapHeadingAlgae()
                            .get()) {
                double targetAngle =
                        mRobotState.getLatestFieldToRobot().getValue().getX()
                                        > FieldConstants.fieldLength / 2.0
                                ? 0
                                : Math.PI;
                if (Math.abs(
                                mRobotState.getLatestFieldToRobot().getValue().getX()
                                        - FieldConstants.fieldLength / 2.0)
                        > 1.0) {
                    mHeadingSetpoint = Optional.of(new Rotation2d(targetAngle));
                }
                Logger.recordOutput("DriveMaintainHeading/bargeHeadingLock", targetAngle);
                mDrivetrain.setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(mHeadingSetpoint.get()));

                // processor heading lock
            } else if (mRobotContainer.getModalControls().algaeClimbMode().getAsBoolean()
                    && mRobotContainer
                                    .getModalSuperstructureTriggers()
                                    .getLatestAlgaeStageState()
                                    .get()
                            == SuperstructureState.STAGE_PROCESSOR
                    && mRobotContainer
                            .getModalSuperstructureTriggers()
                            .getEnableSnapHeadingAlgae()
                            .get()) {
                double targetAngle =
                        mRobotContainer.getRobotState().isRedAlliance()
                                ? Math.PI / 2
                                : -Math.PI / 2;
                if (mRobotContainer.getRobotState().onOpponentSide()) {
                    targetAngle += Math.PI;
                }
                mHeadingSetpoint = Optional.of(new Rotation2d(targetAngle));
                Logger.recordOutput("DriveMaintainHeading/processorHeadingLock", targetAngle);
                mDrivetrain.setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(mHeadingSetpoint.get()));

                // normal maintain heading
            } else {
                mDrivetrain.setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(mHeadingSetpoint.get()));
            }
            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput(
                    "DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
        */
}