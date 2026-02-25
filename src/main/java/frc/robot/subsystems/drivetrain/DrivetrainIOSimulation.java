package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Consumer;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.minolib.swerve.MapleSimulatedSwerveDrivetrain;
import frc.robot.RobotState;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.GlobalConstants;

public class DrivetrainIOSimulation extends DrivetrainIOHardware {
    private static final double kSimulationLoopPeriod = 0.005; // 5 ms
    private Notifier simulationNotifier = null;
    private double lastSimulationTime;

    public MapleSimulatedSwerveDrivetrain simulatedSwerveDrivetrain = null;

    Pose2d lastConsumedPose = null;

    Consumer<SwerveDriveState> simulationTelemetryConsumer = swerveDriveState -> {
        if (GlobalConstants.kUseMapleSim && simulatedSwerveDrivetrain != null) {
            swerveDriveState.Pose = simulatedSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }
        
        telemetryConsumer.accept(swerveDriveState);
    };

    public DrivetrainIOSimulation(RobotState robotState, SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(robotState, driveTrainConstants, modules);

        registerTelemetry(simulationTelemetryConsumer);
        updateSimulationState();
    }

    @Override
    public void updateDrivetrainInputs(DrivetrainIOInputs inputs) {
        super.updateDrivetrainInputs(inputs);

        var pose = getSimulatedPose();
        if (pose != null) {
            Logger.recordOutput("Drivetrain/SimulationPose", pose);
        }
    }

    @SuppressWarnings("unchecked")
    @Override
    public void updateSimulationState() {
        if(GlobalConstants.kUseMapleSim) {
            simulatedSwerveDrivetrain = new MapleSimulatedSwerveDrivetrain(
                Seconds.of(kSimulationLoopPeriod), 
                Kilogram.of(DrivetrainConstants.kRobotMassKilograms), 
                Inches.of(31), 
                Inches.of(31), 
                DCMotor.getKrakenX60Foc(1), 
                DCMotor.getKrakenX44Foc(1), 
                1.2, 
                getModuleLocations(), 
                getPigeon2(), 
                getModules(), 
                SimulationTunerConstants.FrontLeft,
                SimulationTunerConstants.FrontRight,
                SimulationTunerConstants.BackLeft,
                SimulationTunerConstants.BackRight
            );

            simulationNotifier = new Notifier(simulatedSwerveDrivetrain::update);
        } else {
            lastSimulationTime = Utils.getCurrentTimeSeconds();
            simulationNotifier = new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimulationTime;
                lastSimulationTime = currentTime;
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
        }

        simulationNotifier.startPeriodic(kSimulationLoopPeriod);
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        if (GlobalConstants.kUseMapleSim) {
            if (simulatedSwerveDrivetrain != null) {
                getSimulatedDrivetrain().setSimulationWorldPose(pose);
                Timer.delay(0.1);
            }
        }
        
        super.resetOdometry(pose);
    }

    public Pose2d getSimulatedPose() {
        return simulatedSwerveDrivetrain != null ? lastConsumedPose : null;
    }

    public SwerveDriveSimulation getSimulatedDrivetrain() {
        return simulatedSwerveDrivetrain.mapleSimDrive;
    }

}
