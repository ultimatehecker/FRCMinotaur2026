// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.minolib.controller.CommandSimulatedXboxController;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.robot.command_factories.DrivetrainFactory;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIOHardware;
import frc.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import frc.robot.subsystems.drivetrain.SimulationTunerConstants;

public class RobotContainer {
  private final Consumer<WeightedPoseEstimate> visionEstimateConsumer = new Consumer<WeightedPoseEstimate>() {
    @Override
    public void accept(WeightedPoseEstimate estimate) {
        drivetrain.addVisionMeasurement(estimate);
    }
  };

  private final RobotState robotState = new RobotState(visionEstimateConsumer);
  private Drivetrain drivetrain;

  private CommandSimulatedXboxController driverController = new CommandSimulatedXboxController(ControllerConstants.kDriverControllerPort);
  private CommandSimulatedXboxController operatorController = new CommandSimulatedXboxController(ControllerConstants.kOperatorControllerPort);

  private Drivetrain buildDrivetrain() {
    if(Robot.isSimulation()) {
      return new Drivetrain(
        new DrivetrainIOSimulation(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), SimulationTunerConstants.kFrontLeft, SimulationTunerConstants.kFrontRight, SimulationTunerConstants.kBackLeft, SimulationTunerConstants.kBackRight), 
        robotState
      );
    } else {
      return new Drivetrain(
        new DrivetrainIOHardware(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), CompetitionTunerConstants.kFrontLeft, CompetitionTunerConstants.kFrontRight, CompetitionTunerConstants.kBackLeft, CompetitionTunerConstants.kBackRight), 
        robotState
      );
    }
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public RobotContainer() {
    drivetrain = buildDrivetrain();

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(DrivetrainFactory.handleTeleopDrive(
      drivetrain, 
      robotState, 
      driverController::getLeftY, 
      driverController::getLeftX, 
      driverController::getRightX, 
      true
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
