package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.minolib.math.MathUtility;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IndexerConstants;

public class IndexerIOSimulation implements IndexerIO {
    private final DCMotor indexerGearbox;
    private final DCMotorSim indexerSimulation;
    private double indexerAppliedVoltage = 0.0;

    private final PIDController indexerController = new PIDController(0.0, 0.0, 0.0);
    private boolean indexerClosedLoop = false;
    private boolean indexerControllerNeedsReset = false;


    public IndexerIOSimulation() {
        indexerGearbox = IndexerConstants.kIndexerSimulatedGearbox;
        indexerSimulation = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                indexerGearbox,
                IndexerConstants.kIndexerMOI.in(KilogramSquareMeters),
                IndexerConstants.kIndexerMotorReduction
            ),
            indexerGearbox
        );
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            indexerControllerNeedsReset = true;
        }

        inputs.isMotorConnected = true;
        inputs.position = indexerSimulation.getAngularPositionRad();
        inputs.velocity = indexerSimulation.getAngularVelocityRadPerSec();
        inputs.acceleration = 0.0; 
        inputs.appliedVoltage = indexerAppliedVoltage;
        inputs.supplyCurrentAmperes = indexerSimulation.getCurrentDrawAmps();
        inputs.tempuratureCelcius = 0.0; 
    }

    @Override
    public void setVoltage(double voltage) {
        indexerClosedLoop = false;
        indexerAppliedVoltage = MathUtility.clamp(voltage, -12.0, 12.0);        
        indexerSimulation.setInputVoltage(indexerAppliedVoltage);
    }

    @Override
    public void setVelocity(double velocity, double feedforward) {
        if (!indexerClosedLoop) {
            indexerControllerNeedsReset = true;
            indexerClosedLoop = true;
        }
        if (indexerControllerNeedsReset) {
            indexerController.reset();
            indexerControllerNeedsReset = false;
        }

        setVoltage(indexerController.calculate(indexerSimulation.getAngularVelocityRadPerSec(), velocity) + feedforward);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        indexerController.setPID(kP, kI, kD);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void refreshData() {
        indexerSimulation.update(GlobalConstants.kLoopPeriodSeconds);
    }
}
