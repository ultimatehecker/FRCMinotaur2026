// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.zip.ZipEntry;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.controller.driverstation.OperatorButtonBoard;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.robot.command_factories.DrivetrainFactory;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
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
import frc.robot.subsystems.shooter.ShootingPreset;
import frc.robot.subsystems.shooter.flywheel.Shooter;
import frc.robot.subsystems.shooter.flywheel.ShooterIOHardware;
import frc.robot.subsystems.shooter.flywheel.ShooterIOSimulation;
import frc.robot.subsystems.shooter.flywheel.Shooter.SelectedShootingPreset;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOHardware;
import frc.robot.subsystems.shooter.hood.HoodIOSimulation;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.slam.SlamIOSimulation;
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
  private Tower tower;
  private Shooter shooter;
  private Hood hood;
  private Elevator elevator;

  private Controlboard controlboard = Controlboard.getInstance();
  private final OperatorButtonBoard primaryButtonBoard = new OperatorButtonBoard(ControllerConstants.kDriverControllerPort);
  private final OperatorButtonBoard secondaryButtonBoard = new OperatorButtonBoard(ControllerConstants.kOperatorConsolePort);

  // Primary operator panel overrides
  private final Trigger robotRelative = primaryButtonBoard.spstSwitch(11);
  private final Trigger coast = primaryButtonBoard.spstSwitch(10);
  private final Trigger disableAutoShooter = primaryButtonBoard.spstSwitch(9);
  private final Trigger wonAutoOverride = primaryButtonBoard.spstSwitch(8);
  private final Trigger lostAutoOverride = primaryButtonBoard.spstSwitch(8);

  private final LoggedDashboardChooser<Command> autoRegistry;

  private final Alert driverControllerDisconnected = new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorControllerDisconnected = new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);
  private final Alert primaryButtonBoardDisconnected = new Alert("Primary Button Board disconnected (port 2).", AlertType.kWarning);
  private final Alert secondaryButtonBoardDisconnected = new Alert("Primary Button Board disconnected (port 2).", AlertType.kWarning);
  private final Alert autoWinnerNotSet = new Alert("Winner of Autonomous has not been set", AlertType.kError);

  private Drivetrain buildDrivetrain() {
    if(Robot.isSimulation()) {
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

  private Shooter buildShooter() {
    if(Robot.isSimulation()) {
      return new Shooter(
        new ShooterIOSimulation()
      );
    } else {
      return new Shooter(
        new ShooterIOHardware()
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

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Intake getIntake() {
   return intake;
  }

  public RobotContainer() {
    drivetrain = buildDrivetrain();
    intake = buildIntake();
    indexer = buildIndexer();
    tower = buildTower();
    shooter = buildShooter();
    hood = buildHood();
    elevator = buildElevator();

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
      Commands.runOnce(() -> shooter.setShootingPreset(SelectedShootingPreset.CLOSE)).andThen(
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
            () -> shooter.runVoltage(shooter.getPreset().getData().getVoltageSetpoint()), 
            () -> shooter.stop(),
            shooter
          ),
          Commands.runEnd(
              () -> indexer.setGoal(IndexerGoal.FEED), 
              () -> indexer.setGoal(IndexerGoal.STOP), 
              indexer
          ),
          Commands.runEnd(
            () -> hood.setAngle(shooter.getPreset().getData().getHoodAngleDegrees()), 
            () -> hood.setAngle(shooter.getPreset().getData().getHoodAngleDegrees()), 
            hood
          )
        ).raceWith(
          new WaitCommand(4)
        )
      )
    );

    NamedCommands.registerCommand(
      "Shooting OL Sequence Medium", 
      Commands.runOnce(() -> shooter.setShootingPreset(SelectedShootingPreset.MEDIUM)).andThen(
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
            () -> shooter.runVoltage(shooter.getPreset().getData().getVoltageSetpoint()), 
            () -> shooter.stop(),
            shooter
          ),
          Commands.runEnd(
              () -> indexer.setGoal(IndexerGoal.FEED), 
              () -> indexer.setGoal(IndexerGoal.STOP), 
              indexer
          ),
          Commands.runEnd(
            () -> hood.setAngle(shooter.getPreset().getData().getHoodAngleDegrees()), 
            () -> hood.setAngle(shooter.getPreset().getData().getHoodAngleDegrees()), 
            hood
          )
        ).raceWith(
          new WaitCommand(4)
        )
      )
    );

    new EventTrigger("Deploy and Intake")
      .onTrue(IntakeFactory.intakeCommand(this)
      );

    new EventTrigger("Deploy Half and Stop Intaking")
      .onTrue(IntakeFactory.deployHalfCommand(this)
      );
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(DrivetrainFactory.handleTeleopDrive(
      drivetrain, 
      robotState, 
      () -> controlboard.getThrottle(), 
      () -> controlboard.getStrafe(), 
      () -> controlboard.getRotation(), 
      true
    ));
  }

  private void configureDriverBindings() {
    // controlboard
    //   .resetGyro()
    //   .onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

    controlboard
      .deployIntake()
      .whileTrue(IntakeFactory.intakeCommand(this))
      .onFalse(IntakeFactory.deployHalfCommand(this));

    controlboard
      .stowIntake()
      .whileTrue(IntakeFactory.stowCommand(this));

    controlboard.automaticallyShoot()
      .whileTrue(
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
            () -> shooter.runVelocity(shooter.getPreset().getData().getFlywheelSpeedRPM()), 
            () -> shooter.stop(),
            shooter
          ),
          Commands.runEnd(
              () -> indexer.setGoal(IndexerGoal.FEED), 
              () -> indexer.setGoal(IndexerGoal.STOP), 
              indexer
          ),
          Commands.runEnd(
            () -> hood.setAngle(20), 
            () -> hood.setAngle(10), 
            hood
          )
        )
      );

      controlboard
        .exhaust()
        .whileTrue(Commands.parallel(
          Commands.runEnd(
              () -> indexer.setGoal(IndexerGoal.EXHAUST), 
              () -> indexer.setGoal(IndexerGoal.STOP), 
              indexer
          ),
          Commands.runEnd(
              () -> tower.setTowerGoal(TowerGoal.FEED), 
              () -> tower.setTowerGoal(TowerGoal.STOP), 
              tower
          ),
          IntakeFactory.ejectCommand(this)
        ));

      controlboard
        .automaticallyAim()
        .whileTrue(
          DrivetrainFactory.autoAim(drivetrain, robotState, () -> shooter.getHubCenter(robotState.isRedAlliance()), () -> 0, () -> 0)
        );

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
      .onTrue(Commands.runOnce(() -> shooter.setShootingPreset(SelectedShootingPreset.CLOSE)));

    controlboard
      .selectMediumShootingPreset()
      .onTrue(Commands.runOnce(() -> shooter.setShootingPreset(SelectedShootingPreset.MEDIUM)));

    controlboard
      .selectFarShootingPreset()
      .onTrue(Commands.runOnce(() -> shooter.setShootingPreset(SelectedShootingPreset.FAR)));

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
  }

  public Command getAutonomousCommand() {
    return autoRegistry.get();
  }
}
