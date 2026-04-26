package frc.robot.command_factories;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.minolib.utilities.AllianceFlipUtility;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShootingParameters;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.tower.Tower;
import frc.robot.utilities.HubShiftUtility;

public class SuperstructureFactory {
    public static Command shootCommand(Drivetrain drivetrain, Flywheel flywheel, Hood hood, Tower tower, Indexer indexer, Led led, RobotState robotState, DoubleSupplier throttle, DoubleSupplier strafe, BooleanSupplier feedTrigger) {
        return Commands.parallel(
            DrivetrainFactory.autoAim(
                drivetrain,
                robotState,
                () -> AllianceFlipUtility.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
                throttle,
                strafe
            ),
            Commands.sequence(
                Commands.parallel(
                    Commands.run(() -> {
                        ShootingParameters params = ShooterCalculator.getInstance().getParameters();
                        if (params.isValid()) {
                            flywheel.runVelocity(params.flywheelVelocity());
                        } else {
                            flywheel.runVelocity(ShooterCalculator.hubPreset.flywheelSpeed().get());
                        }
                    }, flywheel),

                    Commands.run(() -> {
                        ShootingParameters params = ShooterCalculator.getInstance().getParameters();
                        if (params.isValid()) {
                            hood.setAngle(Math.toDegrees(params.hoodAngle()));
                        } else {
                            hood.stow();
                        }
                    }, hood)
                ).until(() -> {
                    ShootingParameters params = ShooterCalculator.getInstance().getParameters();
                    boolean calculatorReady = params.isValid() && params.confidence() > 60;
                    boolean flywheelReady = flywheel.isReady();
                    boolean hoodReady = hood.isReady();
                    boolean aimed = DrivetrainFactory.isAimed(
                        robotState,
                        AllianceFlipUtility.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                    );

                    boolean shiftActive = HubShiftUtility.getShiftedShiftInfo().active();

                    Logger.recordOutput("Shooting/FlywheelReady", flywheelReady);
                    Logger.recordOutput("Shooting/HoodReady", hoodReady);
                    Logger.recordOutput("Shooting/Aimed", aimed);
                    Logger.recordOutput("Shooting/CalculatorReady", calculatorReady);
                    Logger.recordOutput("Shooting/ShiftActive", shiftActive);
                    Logger.recordOutput("Shooting/Confidence", params.confidence());
                    Logger.recordOutput("Shooting/ReadyToFeed", flywheelReady && hoodReady && aimed && calculatorReady && feedTrigger.getAsBoolean());

                    return flywheelReady && hoodReady && aimed && calculatorReady && feedTrigger.getAsBoolean();
                }),
                Commands.parallel(
                    Commands.run(() -> {
                        ShootingParameters params = ShooterCalculator.getInstance().getParameters();
                        if (params.isValid()) {
                            flywheel.runVelocity(params.flywheelVelocity());
                        }
                    }, flywheel),

                    Commands.run(() -> {
                        ShootingParameters params = ShooterCalculator.getInstance().getParameters();
                        if (params.isValid()) {
                            hood.setAngle(Math.toDegrees(params.hoodAngle()));
                        }
                    }, hood),

                    Commands.run(() -> tower.setTowerGoal(Tower.TowerGoal.FEED), tower),
                    Commands.run(() -> indexer.setGoal(Indexer.IndexerGoal.FEED), indexer)
                ).until(() -> !feedTrigger.getAsBoolean())
            ),
            Commands.run(() -> {}, led).deadlineFor(
                Commands.repeatingSequence(
                    Commands.defer(() -> {
                        boolean flywheelReady = flywheel.isReady();
                        boolean hoodReady = hood.isReady();
                        boolean aimed = DrivetrainFactory.isAimed(
                            robotState,
                            AllianceFlipUtility.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                        );

                        boolean feeding = tower.getTowerGoal() == Tower.TowerGoal.FEED;

                        if (feeding) {
                            return led.commandBlink(Color.kGreen, Seconds.of(0.15));
                        } else if (flywheelReady && hoodReady && aimed) {
                            return led.commandStaticColor(Color.kGreen);
                        } else {
                            return led.commandBlink(Color.kYellow, Seconds.of(0.15));
                        }
                    }, Set.of(led))
                )
            )
        )
        .finallyDo(interrupted -> {
            tower.setTowerGoal(Tower.TowerGoal.STOP);
            indexer.setGoal(Indexer.IndexerGoal.IDLE);
            flywheel.runVelocity(400);
            hood.stow();
        }).withName("Shoot SOTM");
    }

    public static Command shootStationaryCommand(Drivetrain drivetrain, Flywheel shooter, Hood hood, Tower tower, Indexer indexer, Led led, RobotState robotState, BooleanSupplier feedTrigger) {
        return shootCommand(
            drivetrain, shooter, hood, tower, indexer, led,
            robotState,
            () -> 0.0,
            () -> 0.0,
            feedTrigger
        );
    }
}
