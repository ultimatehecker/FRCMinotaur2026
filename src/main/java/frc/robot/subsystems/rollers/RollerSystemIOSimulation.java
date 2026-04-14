package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;

public class RollerSystemIOSimulation implements RollerSystemIO {
    private final DCMotor rollerGearbox;
    private final DCMotorSim rollerSimulation;
    private double rollerAppliedVoltage = 0.0;

    public RollerSystemIOSimulation() {
        rollerGearbox = IntakeConstants.kRollerSimulatedGearbox;
        rollerSimulation = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                rollerGearbox,
                IntakeConstants.kRollerMOI.in(KilogramSquareMeters),
                IntakeConstants.kRollerMotorReduction
            ),
            rollerGearbox
        );
    }

    @Override
    public void updateInputs(RollerSystemIOInputs inputs) {
        inputs.isMotorConnected = true;
        inputs.positionRadians = rollerSimulation.getAngularPositionRad();
        inputs.velocityRadiansPerSecond = rollerSimulation.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = rollerAppliedVoltage;
        inputs.supplyCurrentAmperes = rollerSimulation.getCurrentDrawAmps();
        inputs.torqueCurrentAmperes = rollerGearbox.getCurrent(rollerSimulation.getAngularVelocityRadPerSec(), rollerAppliedVoltage);
        inputs.temperatureCelsius = 0.0; 
        inputs.temperatureFault = false;
    }

    @Override
    public void setVoltage(double voltage) {
        rollerAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        rollerSimulation.setInputVoltage(rollerAppliedVoltage);
    }

    @Override
    public void setOL(double amperes) {
        setVoltage(rollerGearbox.getVoltage(
            rollerGearbox.getTorque(amperes),
            rollerSimulation.getAngularVelocityRadPerSec()
        ));
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void refreshData() {
        rollerSimulation.update(GlobalConstants.kLoopPeriodSeconds);
    }
}