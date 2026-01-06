package frc.robot.subsystems.drivetrain;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
            moduleMap.put("driveTempuratureCelcius", driveMotor.getDeviceTemp());

            moduleMap.put("steerSupplyCurrentAmperes", steerMotor.getSupplyCurrent());
            moduleMap.put("steerStatorCurrentAmperes", steerMotor.getStatorCurrent());
            moduleMap.put("steerAppliedVoltage", steerMotor.getMotorVoltage());
            moduleMap.put("steerTempuratureCelcius", steerMotor.getDeviceTemp());
        }

        this.cor = new Translation2d(0, 0);
        this.targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }
}