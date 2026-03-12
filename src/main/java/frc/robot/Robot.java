// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;
import com.google.flatbuffers.Constants;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.minolib.advantagekit.LocalADStarAK;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.hardware.MinoCANBus;
import frc.robot.command_factories.DrivetrainFactory;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.RobotType;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  //private static final double loopOverrunWarningTimeout = 0.2;
  //private static final double canErrorTimeThreshold = 0.5; 
  //private static final double canivoreErrorTimeThreshold = 0.5;
  //private static final double lowBatteryVoltage = 11.8;
  //private static final double lowBatteryDisabledTime = 1.5;
  //private static final double lowBatteryMinCycleCount = 10;
  //private static int lowBatteryCycleCount = 0;

  //private final Timer canInitialErrorTimer = new Timer();
  //private final Timer canErrorTimer = new Timer();
  //private final Timer canivoreErrorTimer = new Timer();
  //private final Timer disabledTimer = new Timer();
  //private MinoCANBus canivoreBus;

  //private final Alert canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  //private final Alert canivoreErrorAlert = new Alert("CANivore errors detected, robot may not be controllable.", AlertType.kError);
  //private final Alert lowBatteryAlert = new Alert("Battery voltage is very low, consider turning off the robot or replacing the battery.", AlertType.kWarning);
  //private final Alert jitAlert = new Alert("Please wait to enable, JITing in progress.", AlertType.kWarning);
  //private final Alert noAutoSelectedAlert = new Alert("No auto selected: please select an auto", AlertType.kWarning);

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
      "GitDirty",
      switch(BuildConstants.DIRTY) {
        case 0 -> "All changes committed";
        case 1 -> "Uncommitted changes";
        default -> "Unknown";
    });

    switch(GlobalConstants.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        if(GlobalConstants.getRobot() == RobotType.COMPBOT) {
          //LoggedPowerDistribution.getInstance(0, ModuleType.kCTRE)
        }

        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false);
        String path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    SignalLogger.enableAutoLogging(false); // might disable later
    Logger.start();

    /*
    Pathfinding.setPathfinder(new LocalADStarAK());

    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
      String name = command.getName();
      int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
      commandCounts.put(name, count);
      
      Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
      Logger.recordOutput("CommandsAll/" + name, count > 0);
    };

    CommandScheduler.getInstance().onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    if (GlobalConstants.getMode() == GlobalConstants.Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    // Configure brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    robotContainer = new RobotContainer();
    canivoreBus = GlobalConstants.kCANivoreBus;

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    */

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    //LoggedTracer.reset();

    CommandScheduler.getInstance().run();
    //LoggedTracer.record("Commands");

    /* 
    var canStatus = RobotController.getCANStatus();
    Logger.recordOutput("CANStatus/OffCount", canStatus.busOffCount);
    Logger.recordOutput("CANStatus/TxFullCount", canStatus.txFullCount);
    Logger.recordOutput("CANStatus/ReceiveErrorCount", canStatus.receiveErrorCount);
    Logger.recordOutput("CANStatus/TransmitErrorCount", canStatus.transmitErrorCount);

    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }

    canErrorAlert.set(!canErrorTimer.hasElapsed(GlobalConstants.kCANErrorTimeThreshold) && canInitialErrorTimer.hasElapsed(GlobalConstants.kCANErrorTimeThreshold));
    canivoreBus.updateInputs();

    lowBatteryCycleCount += 1;
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }

    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage && disabledTimer.hasElapsed(lowBatteryDisabledTime) && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
      lowBatteryAlert.set(true);
    }

    // JIT alert
    jitAlert.set(isJITing());

    */

    //LoggedTracer.record("RobotPeriodic");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static boolean isJITing() {
    return Timer.getTimestamp() < 45.0;
  }
}
