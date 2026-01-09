package frc.robot.subsystems.drivetrain;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DrivetrainIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DrivetrainIO {
    private HashMap<String, BaseStatusSignal> frontLeftSignals = new HashMap<>();
    private HashMap<String, BaseStatusSignal> frontRightSignals = new HashMap<>();
    private HashMap<String, BaseStatusSignal> backLeftSignals = new HashMap<>();
    private HashMap<String, BaseStatusSignal> backRightSignals = new HashMap<>();

    private Map<Integer, HashMap<String, BaseStatusSignal>> signalsMap = new HashMap<>();
    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

    private Translation2d cor;
    private ChassisSpeeds targetChassisSpeeds;

    public DrivetrainIOHardware(SwerveDrivetrainConstants constants, SwerveModuleConstants<?, ?, ?>... moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);

        signalsMap.put(0, frontLeftSignals);
        signalsMap.put(1, frontRightSignals);
        signalsMap.put(2, backLeftSignals);
        signalsMap.put(3, backRightSignals);

        for(int i = 0; i < 4; i++) {
            CommonTalon driveMotor = this.getModule(i).getDriveMotor();
            CommonTalon steerMotor = this.getModule(i).getSteerMotor();

            var moduleMap = signalsMap.get(i);

            moduleMap.put("driveSupplyCurrentAmperes", driveMotor.getSupplyCurrent());
            moduleMap.put("driveStatorCurrentAmperes", driveMotor.getStatorCurrent());
            moduleMap.put("driveAppliedVoltage", driveMotor.getMotorVoltage());
            moduleMap.put("driveTemperatureCelsius", driveMotor.getDeviceTemp());

            moduleMap.put("steerSupplyCurrentAmperes", steerMotor.getSupplyCurrent());
            moduleMap.put("steerStatorCurrentAmperes", steerMotor.getStatorCurrent());
            moduleMap.put("steerAppliedVoltage", steerMotor.getMotorVoltage());
            moduleMap.put("steerTemperatureCelsius", steerMotor.getDeviceTemp());
        }

        this.cor = new Translation2d(0, 0);
        this.targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }

     @Override
    public void updateDrivetrainInputs(DrivetrainIOInputs inputs) {
        SwerveDriveState state = this.getStateCopy();
        inputs.logState(state);

        ChassisSpeeds measuredRobotRelativeChassisSpeeds = getKinematics().toChassisSpeeds(inputs.currentModuleStates);
        ChassisSpeeds measuredFieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobotRelativeChassisSpeeds, inputs.Pose.getRotation());
        ChassisSpeeds desiredRobotRelativeChassisSpeeds = getKinematics().toChassisSpeeds(inputs.referenceModuleStates);
        ChassisSpeeds desiredFieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(desiredRobotRelativeChassisSpeeds, inputs.Pose.getRotation());

        inputs.measuredRobotRelativeChassisSpeeds = measuredRobotRelativeChassisSpeeds;
        inputs.measuredFieldRelativeChassisSpeeds = measuredFieldRelativeChassisSpeeds;
        inputs.referenceRobotRelativeChassisSpeeds = desiredRobotRelativeChassisSpeeds;
        inputs.referenceFieldRelativeChassisSpeeds = desiredFieldRelativeChassisSpeeds;
    }

    @Override
    public void updateModuleInputs(ModuleIOInputs... inputs) {
        for (int i = 0; i < 4; i++) {
            var moduleMap = signalsMap.get(i);

            inputs[i].driveSupplyCurrentAmperes = moduleMap.get("driveSupplyCurrentAmperes").getValueAsDouble();
            inputs[i].driveStatorCurrentAmperes = moduleMap.get("driveStatorCurrentAmperes").getValueAsDouble();
            inputs[i].driveAppliedVoltage = moduleMap.get("driveAppliedVoltage").getValueAsDouble();
            inputs[i].driveTemperatureCelsius = moduleMap.get("driveTemperatureCelsius").getValueAsDouble();

            inputs[i].steerSupplyCurrentAmperes = moduleMap.get("steerSupplyCurrentAmperes").getValueAsDouble();
            inputs[i].steerStatorCurrentAmperes = moduleMap.get("steerStatorCurrentAmperes").getValueAsDouble();
            inputs[i].steerAppliedVoltage = moduleMap.get("steerAppliedVoltage").getValueAsDouble();
            inputs[i].steerTemperatureCelsius = moduleMap.get("steerTemperatureCelsius").getValueAsDouble();

        }
    }

    @Override
    public Translation2d getCOR() {
        return cor;
    }

    @Override
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return Commands.run(() -> super.setControl(requestSupplier.get()), subsystemRequired);
    }

    @Override
    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        // Change the neutral mode configuration in a separate thread since changing the configuration of a CTRE device may take a significant amount of time (~200 ms).
        brakeModeExecutor.execute(() -> {
            for (SwerveModule<TalonFX, TalonFX, CANcoder> swerveModule : super.getModules()) {
                swerveModule.getDriveMotor().setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast, 0.25);
                swerveModule.getSteerMotor().setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast, 0.25);
            }
        });
    }

    @Override
    public void setTargetChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
        this.targetChassisSpeeds = targetChassisSpeeds;
    }

    @Override
    public void addVisionMeasurement(VisionPoseEstimate visionFieldPoseEstimate) {
        if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
            this.addVisionMeasurement(visionFieldPoseEstimate.getVisionRobotPoseMeters(), Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()));
        } else {
            this.addVisionMeasurement(
                visionFieldPoseEstimate.getVisionRobotPoseMeters(),
                Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()),
                visionFieldPoseEstimate.getVisionMeasurementStdDevs()
            );
        }
    }

    @Override
    public void setStateStandardDeviations(double xStd, double yStd, double rotStd) {
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(xStd, yStd, rotStd);
        this.setStateStdDevs(stateStdDevs);
    }

    @Override
    public void setCOR(Translation2d cor) {
        this.cor = cor;
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        this.resetPose(pose);
    }

    @Override
    public void refreshData() {
        for (int i = 0; i < 4; i++) {
            var moduleMap = signalsMap.get(i);
            BaseStatusSignal.refreshAll(moduleMap.values().toArray(new BaseStatusSignal[] {}));
        }
    }
}