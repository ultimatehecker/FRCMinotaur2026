// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.Consumer;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.minolib.controller.driverstation.OperatorButtonBoard;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.minolib.utilities.AllianceFlipUtility;
import frc.robot.command_factories.DrivetrainFactory;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.command_factories.SuperstructureFactory;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.Mode;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TowerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.io.Controlboard;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIOHardware;
import frc.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import frc.robot.subsystems.drivetrain.SimulationTunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.rollers.RollerSystemIOSimulation;
import frc.robot.subsystems.rollers.RollerSystemIOHardware;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOHardware;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSimulation;
import frc.robot.subsystems.shooter.flywheel.Flywheel.SelectedShootingPreset;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOHardware;
import frc.robot.subsystems.shooter.hood.HoodIOSimulation;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.slam.SlamIOSimulation;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedIOHardware;
import frc.robot.subsystems.intake.slam.SlamIOHardware;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.Tower.TowerGoal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSimulation;
import frc.robot.utilities.HubShiftUtility;

public class RobotContainer {
  private final Consumer<WeightedPoseEstimate> visionEstimateConsumer = new Consumer<WeightedPoseEstimate>() {
    @Override
    public void accept(WeightedPoseEstimate estimate) {
        drivetrain.addVisionMeasurement(estimate);
    }
  };

  private final RobotState robotState = new RobotState(visionEstimateConsumer);
  private Drivetrain drivetrain;
  private Intake intake;
  private Indexer indexer;
  private Led led;
  private Tower tower;
  private Flywheel flywheel;
  private Hood hood;
  private Elevator elevator;
  private Vision vision;

  private SwerveDriveSimulation driveSimulation = null;

  private Controlboard controlboard = Controlboard.getInstance();
  private final OperatorButtonBoard primaryButtonBoard = new OperatorButtonBoard(ControllerConstants.kDriverControllerPort);
  private final OperatorButtonBoard secondaryButtonBoard = new OperatorButtonBoard(ControllerConstants.kOperatorConsolePort);

  // Primary operator panel overrides
  private final Trigger robotRelative = primaryButtonBoard.spstSwitch(11);
  private final Trigger coast = primaryButtonBoard.spstSwitch(10);
  private final Trigger disableAutoShooter = primaryButtonBoard.spstSwitch(9);
  private final Trigger wonAutoOverride = primaryButtonBoard.spstSwitch(8);
  private final Trigger lostAutoOverride = primaryButtonBoard.spstSwitch(7);

  private final LoggedDashboardChooser<Command> autoRegistry;

  private final Alert driverControllerDisconnected = new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorControllerDisconnected = new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);
  private final Alert primaryButtonBoardDisconnected = new Alert("Primary Button Board disconnected (port 2).", AlertType.kWarning);
  private final Alert secondaryButtonBoardDisconnected = new Alert("Primary Button Board disconnected (port 2).", AlertType.kWarning);

  private boolean coastOverride = false;

  private Drivetrain buildDrivetrain() {
    if(Robot.isSimulation()) {
      this.driveSimulation = new SwerveDriveSimulation(DrivetrainConstants.kMapleSimConfiguration, new Pose2d(3, 3, new Rotation2d()));
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

      return new Drivetrain(
        new DrivetrainIOSimulation(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), SimulationTunerConstants.kFrontLeft, SimulationTunerConstants.kFrontRight, SimulationTunerConstants.kBackLeft, SimulationTunerConstants.kBackRight), 
        robotState
      );
    } else {
      return new Drivetrain(
        new DrivetrainIOHardware(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), CompetitionTunerConstants.FrontLeft, CompetitionTunerConstants.FrontRight, CompetitionTunerConstants.BackLeft, CompetitionTunerConstants.BackRight), 
        robotState
      );
    }
  }

  private Intake buildIntake() {
   if(Robot.isSimulation()) {
     return new Intake(
       new SlamIOSimulation(),
       new RollerSystemIOSimulation()
        //robotState
     );
   } else {
     return new Intake(
      new SlamIOHardware(),
      new RollerSystemIOHardware(IntakeConstants.kRollerMotor, IntakeConstants.kRollerMotorSupplyLimit.in(Amps), IntakeConstants.kRollerMotorInverted, true, IntakeConstants.kRollerMotorReduction)
     );
   }
  }

  private Indexer buildIndexer() {
    if(Robot.isSimulation()) {
      return new Indexer(
        new RollerSystemIOSimulation(),
        new RollerSystemIOSimulation()
      );
    } else {
      return new Indexer( 
        new RollerSystemIOHardware(IndexerConstants.kLeftMotor, IndexerConstants.kMotorSupplyLimit.in(Amps), false, true, IndexerConstants.kRollerReduction),
        new RollerSystemIOHardware(IndexerConstants.kRightMotor, IndexerConstants.kMotorSupplyLimit.in(Amps), true, true, IndexerConstants.kRollerReduction)
      );
    }
  }

  private Led buildLed() {
    return new Led(new LedIOHardware(), robotState);
  }

  private Tower buildTower() {
    if(Robot.isSimulation()) {
      return new Tower(
        new RollerSystemIOSimulation(),
        new RollerSystemIOSimulation()
      );
    } else {
      return new Tower(
        new RollerSystemIOHardware(TowerConstants.kTopMotor, TowerConstants.kTopRollerSupplyLimit.in(Amps), true, true, TowerConstants.kTopMotorReduction),
        new RollerSystemIOHardware(TowerConstants.kBottomMotor, TowerConstants.kBottomRollerSupplyLimit.in(Amps), true, false, TowerConstants.kBottomMotorReduction)
      );
    }
  }

  private Flywheel buildShooter() {
    if(Robot.isSimulation()) {
      return new Flywheel(
        new FlywheelIOSimulation()
      );
    } else {
      return new Flywheel(
        new FlywheelIOHardware()
      );
    }
  }

  private Hood buildHood() {
    if(Robot.isSimulation()) {
      return new Hood(
        new HoodIOSimulation()
      );
    } else {
      return new Hood(
        new HoodIOHardware()
      );
    }
  }

  private Elevator buildElevator() {
    if(Robot.isSimulation()) {
      return new Elevator(
        new ElevatorIOHardware()
      );
    } else {
      return new Elevator(
        new ElevatorIOHardware()
      );
    }
  }

  private Vision buildVision() {
    if(Robot.isSimulation()) {
      return new Vision(
        robotState, 
        new VisionIOSimulation(VisionConstants.kBackLeftConfiguration, VisionConstants.kAprilTagLayout, robotState)
      );
    } else {
      return new Vision(
        robotState, 
        new VisionIOPhotonVision(VisionConstants.kBackLeftConfiguration, VisionConstants.kAprilTagLayout)
      );
    }
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Intake getIntake() {
   return intake;
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public RobotContainer() {
    drivetrain = buildDrivetrain();
    intake = buildIntake();
    indexer = buildIndexer();
    led = buildLed();
    tower = buildTower();
    flywheel = buildShooter();
    hood = buildHood();
    elevator = buildElevator();
    vision = buildVision();

    intake.setBrakeMode(() -> !coastOverride);
    indexer.setBrakeMode(() -> !coastOverride);
    tower.setBrakeMode(() -> !coastOverride);
    flywheel.setBrakeMode(() -> !coastOverride);
    hood.setBrakeMode(() -> !coastOverride);
    elevator.setBrakeMode(() -> !coastOverride);

    HubShiftUtility.setAllianceWinOverride(() -> {
      if (lostAutoOverride.getAsBoolean()) {
        return Optional.of(false);
      }

      if (wonAutoOverride.getAsBoolean()) {
        return Optional.of(true);
      }

      return Optional.empty();
    });

    configureNamedCommands();

    autoRegistry = new LoggedDashboardChooser<Command>("Auton Choices", AutoBuilder.buildAutoChooser());
    autoRegistry.addOption("Drivetrain Translation Dynamic Forward", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Dynamic Reverse", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Forward", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Reverse", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    autoRegistry.addOption("Do Nothing", Commands.none());
    
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
      "Shooting OL Sequence Close", 
      Commands.runOnce(() -> flywheel.setShootingPreset(SelectedShootingPreset.CLOSE)).andThen(
        Commands.parallel(
          Commands.sequence(
            Commands.waitSeconds(1),
            Commands.runEnd(
              () -> tower.setTowerGoal(TowerGoal.FEED), 
              () -> tower.setTowerGoal(TowerGoal.STOP), 
              tower
            )
          ),
          Commands.runEnd(
            () -> flywheel.runVoltage(flywheel.getPreset().getData().getVoltageSetpoint()), 
            () -> flywheel.stop(),
            flywheel
          ),
          Commands.runEnd(
              () -> indexer.setGoal(IndexerGoal.FEED), 
              () -> indexer.setGoal(IndexerGoal.STOP), 
              indexer
          ),
          Commands.runEnd(
            () -> hood.setAngle(flywheel.getPreset().getData().getHoodAngleDegrees()), 
            () -> hood.setAngle(12.0), 
            hood
          )
        ).raceWith(
          new WaitCommand(4)
        )
      )
    );

    NamedCommands.registerCommand(
      "Shooting OL Sequence Medium", 
      Commands.runOnce(() -> flywheel.setShootingPreset(SelectedShootingPreset.MEDIUM)).andThen(
        Commands.parallel(
          Commands.sequence(
            Commands.waitSeconds(1),
            Commands.runEnd(
              () -> tower.setTowerGoal(TowerGoal.FEED), 
              () -> tower.setTowerGoal(TowerGoal.STOP), 
              tower
            )
          ),
          Commands.runEnd(
            () -> flywheel.runVoltage(flywheel.getPreset().getData().getVoltageSetpoint()), 
            () -> flywheel.stop(),
            flywheel
          ),
          Commands.runEnd(
              () -> indexer.setGoal(IndexerGoal.FEED), 
              () -> indexer.setGoal(IndexerGoal.STOP), 
              indexer
          ),
          Commands.runEnd(
            () -> hood.setAngle(flywheel.getPreset().getData().getHoodAngleDegrees()), 
            () -> hood.setAngle(12.0), 
            hood
          )
        ).raceWith(
          new WaitCommand(4)
        )
      )
    );

    new EventTrigger("Deploy and Intake")
      .onTrue(IntakeFactory.intakeCommandUnblocking(this)
    );

    new EventTrigger("Deploy Half and Stop Intaking")
      .onTrue(IntakeFactory.deployHalfCommand(this)
    );

    new EventTrigger("Stow and Stop Intaking")
      .onTrue(IntakeFactory.stowCommand(this)
    );
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(DrivetrainFactory.handleTeleopDrive(
      drivetrain, 
      robotState, 
      () -> controlboard.getThrottle(), 
      () -> controlboard.getStrafe(), 
      () -> controlboard.getRotation(),
      () -> true
    ));
  }

  private void configureDriverBindings() {
    controlboard
      .resetGyro()
      .onTrue(Commands.runOnce(() -> drivetrain.resetOdometry(new Pose2d(robotState.getLatestFieldToRobot().getValue().getTranslation(), AllianceFlipUtility.apply(Rotation2d.kZero)))));

    controlboard
      .deployIntake()
      .whileTrue(
        IntakeFactory.intakeCommand(this)
      )
      .onFalse(IntakeFactory.deployHalfCommand(this));

    controlboard
      .stowIntake()
      .whileTrue(IntakeFactory.stowCommand(this));

    controlboard.automaticallyShoot()
      .whileTrue(
        Commands.parallel(
          Commands.sequence(
            Commands.waitSeconds(1),
            Commands.runOnce(
              () -> tower.setTowerGoal(TowerGoal.FEED)
            ),
            Commands.runOnce(
              () -> indexer.setGoal(IndexerGoal.FEED)
            ),
            IntakeFactory.feedCommand(this),
            Commands.waitSeconds(0.2),
            IntakeFactory.stowCommand(this),
            Commands.waitSeconds(0.2),
            IntakeFactory.feedCommand(this),
            Commands.waitSeconds(0.2),
            IntakeFactory.stowCommand(this),
            Commands.waitSeconds(0.2),
            IntakeFactory.feedCommand(this),
            Commands.waitSeconds(0.2),
            IntakeFactory.stowCommand(this),
            Commands.waitSeconds(0.2)
          ),
          Commands.runEnd(
            () -> flywheel.runVelocity(flywheel.getPreset().getData().getFlywheelSpeedRPM()), 
            () -> flywheel.stop(),
            flywheel
          ),
          Commands.runEnd(
            () -> hood.setAngle(flywheel.getPreset().getData().getHoodAngleDegrees()), //TODO: Change this later before the practice field, 
            () -> hood.setAngle(12), 
            hood
          )
        ).finallyDo(() -> tower.setTowerGoal(TowerGoal.STOP)).alongWith(Commands.runOnce(() -> indexer.setGoal(IndexerGoal.STOP)))
      );

      controlboard
        .automaticallyAim()
        .whileTrue(DrivetrainFactory.autoAim(drivetrain, robotState, () -> AllianceFlipUtility.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()), () -> 0.0, () -> 0.0));

      controlboard
        .exhaust()
        .whileTrue(Commands.parallel(
          Commands.runEnd(
              () -> indexer.setGoal(IndexerGoal.EXHAUST), 
              () -> indexer.setGoal(IndexerGoal.STOP), 
              indexer
          ),
          Commands.runEnd(
              () -> tower.setTowerGoal(TowerGoal.EXHAUST), 
              () -> tower.setTowerGoal(TowerGoal.STOP), 
              tower
          ),
          IntakeFactory.ejectCommand(this)
        ));

      controlboard
        .automaticallyHang()
        .whileTrue(
          DrivetrainFactory.driveToPoint(
            drivetrain, 
            robotState, 
            () -> new Pose2d(1, 2.8, new Rotation2d(Math.PI)), 
            DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond), 
            DrivetrainConstants.kMaximumRotationalVelocity.in(RadiansPerSecond)
          )
        );
  }

  public void updateOnboardAlerts() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    SmartDashboard.putString("Shifts/Remaining Shift Time", String.format("%.1f", Math.max(HubShiftUtility.getShiftedShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtility.getShiftedShiftInfo().active());
    SmartDashboard.putString("Shifts/Game State", HubShiftUtility.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean("Shifts/Active First?", DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtility.getFirstActiveAlliance());

    driverControllerDisconnected.set(!DriverStation.isJoystickConnected(controlboard.getPrimaryHID().getPort()));
    operatorControllerDisconnected.set(!DriverStation.isJoystickConnected(controlboard.getSecondaryHID().getPort()));
    primaryButtonBoardDisconnected.set(!primaryButtonBoard.isConnected());
    secondaryButtonBoardDisconnected.set(!secondaryButtonBoard.isConnected());
  }

  private void configureOperatorBindings() {
    controlboard
      .selectCloseShootingPreset()
      .onTrue(Commands.runOnce(() -> flywheel.setShootingPreset(SelectedShootingPreset.CLOSE)));

    controlboard
      .selectMediumShootingPreset()
      .onTrue(Commands.runOnce(() -> flywheel.setShootingPreset(SelectedShootingPreset.MEDIUM)));

    controlboard
      .selectFarShootingPreset()
      .onTrue(Commands.runOnce(() -> flywheel.setShootingPreset(SelectedShootingPreset.FAR)));

    controlboard
      .deployClimber()
      .whileTrue(Commands.runEnd(
        () -> elevator.runVoltage(3), 
        () -> elevator.runVoltage(0), 
        elevator
      ));

    controlboard
      .stowClimber()
      .whileTrue(Commands.runEnd(
        () -> elevator.runVoltage(-3), 
        () -> elevator.runVoltage(0), 
        elevator
      ));

      coast
        .onTrue(Commands.runOnce(() -> {
          if (DriverStation.isDisabled()) {
            coastOverride = true;
          }
        }).alongWith(led.commandStaticColor(Color.kWhite).until(() -> !coast.getAsBoolean()))
          .withName("Superstructure Coast")
          .ignoringDisable(true)
        )
        .onFalse(Commands.runOnce(() -> {
          coastOverride = false;
        })
          .withName("Superstructure Uncoast")
          .ignoringDisable(true)
        );
  }

  public void resetSimulationField() {
    if (GlobalConstants.getMode() != Mode.SIM) return;

    drivetrain.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (GlobalConstants.getMode() != Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  public Command getAutonomousCommand() {
    return autoRegistry.get();
  }
}
