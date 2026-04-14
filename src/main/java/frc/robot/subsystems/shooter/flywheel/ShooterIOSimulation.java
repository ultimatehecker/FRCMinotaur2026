package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterIOSimulation implements ShooterIO {
    private final DCMotor shooterGearbox;
    private final FlywheelSim shooterSimulation;
    private double shooterAppliedVoltage = 0.0;

    private final PIDController shooterController = new PIDController(0.0, 0.0, 0.0);
    private boolean shooterClosedLoop = false;
    private boolean shooterControllerNeedsReset = false;

    public ShooterIOSimulation() {
        shooterGearbox = ShooterConstants.kShooterSimulatedGearbox;
        shooterSimulation = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                shooterGearbox,
                ShooterConstants.kShooterMOI.in(KilogramSquareMeters),
                ShooterConstants.kShooterMotorReduction
            ),
            shooterGearbox
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            shooterControllerNeedsReset = true;
        }

        inputs.isFirstShooterConnected = true;
        inputs.firstShooterPosition = 0.0;  
        inputs.firstShooterVelocity = shooterSimulation.getAngularVelocityRadPerSec();
        inputs.firstShooterAppliedVoltage = shooterAppliedVoltage;
        inputs.firstShooterSupplyCurrentAmperes = shooterSimulation.getCurrentDrawAmps();
        inputs.firstShooterTorqueCurrentAmperes = shooterGearbox.getCurrent(shooterSimulation.getAngularVelocityRadPerSec(), shooterAppliedVoltage);
        inputs.firstShooterTempuratureCelcius = 0.0;

        inputs.isSecondShooterConnected = true;
        inputs.secondShooterSupplyCurrentAmperes = shooterSimulation.getCurrentDrawAmps();
        inputs.secondShooterTempuratureCelcius = 0.0;
        inputs.secondShooterTempuratureFault = false;

        inputs.isThirdShooterConnected = true;
        inputs.thirdShooterSupplyCurrentAmperes = shooterSimulation.getCurrentDrawAmps();
        inputs.thirdShooterTempuratureCelcius = 0.0;
        inputs.secondShooterTempuratureFault = false;

        inputs.isFourthShooterConnected = true;
        inputs.fourthShooterSupplyCurrentAmperes = shooterSimulation.getCurrentDrawAmps();
        inputs.fourthShooterTempuratureCelcius = 0.0;
        inputs.fourthShooterTempuratureFault = false;
    }

    @Override
    public void setVoltage(double voltage) {
        shooterClosedLoop = false;
        shooterAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        shooterSimulation.setInputVoltage(shooterAppliedVoltage);
    }

    @Override
    public void setVelocity(double velocity, double feedforward) {
        if (!shooterClosedLoop) {
            shooterControllerNeedsReset = true;
            shooterClosedLoop = true;
        }
        
        if (shooterControllerNeedsReset) {
            shooterController.reset();
            shooterControllerNeedsReset = false;
        }
        
        setVoltage(shooterController.calculate(shooterSimulation.getAngularVelocityRadPerSec(), velocity) +  feedforward);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        shooterController.setPID(kP, kI, kD);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void refreshData() {
        shooterSimulation.update(GlobalConstants.kLoopPeriodSeconds);
    }
}
