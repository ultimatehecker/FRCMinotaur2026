package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.minolib.math.MathUtility;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.HoodConstants;

public class HoodIOSimulation implements HoodIO {
    private final DCMotor hoodGearbox;
    private final SingleJointedArmSim hoodSimulation;
    private double hoodAppliedVoltage = 0.0;

    private final PIDController hoodPositionController = new PIDController(0.0, 0.0, 0.0);
    private boolean hoodClosedLoop = false;
    private boolean hoodControllerNeedsReset = false;

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(
        HoodConstants.kHoodLength.in(Meters) * 3,
        HoodConstants.kHoodLength.in(Meters) * 3
    );

    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("Pivot Joint", HoodConstants.kHoodLength.in(Meters) * 1.5, HoodConstants.kHoodLength.in(Meters) * 1.5);

    private final LoggedMechanismLigament2d pivotLigament = mechanismRoot.append(
        new LoggedMechanismLigament2d(
            "Arm",
            HoodConstants.kHoodLength.in(Meters),
            HoodConstants.kHoodMinimumPosition.in(Degrees),
            4,
            new Color8Bit(Color.kMaroon)
        )
    );

    public HoodIOSimulation() {
        hoodGearbox = HoodConstants.kSimulatedGearbox;
        hoodSimulation = new SingleJointedArmSim(
            hoodGearbox,
            HoodConstants.kMotorReduction,
            HoodConstants.kHoodMOI.in(KilogramSquareMeters),
            HoodConstants.kHoodLength.in(Meters),
            HoodConstants.kHoodMinimumPosition.in(Radians),
            HoodConstants.kHoodMaximumPosition.in(Radians),
            true, 
            HoodConstants.kHoodStartingPosition.in(Radians)
        );
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            hoodControllerNeedsReset = true;
        }

        inputs.isMotorConnected = true;
        inputs.positionRadians = hoodSimulation.getAngleRads();
        inputs.velocityRadiansPerSecond = hoodSimulation.getVelocityRadPerSec();
        inputs.appliedVoltage = hoodAppliedVoltage;
        inputs.torqueCurrentAmperes = hoodGearbox.getCurrent(hoodSimulation.getVelocityRadPerSec(), hoodAppliedVoltage);
        inputs.supplyCurrentAmperes = hoodSimulation.getCurrentDrawAmps();
        inputs.temperatureCelsius = 0.0; 
        inputs.temperatureFault = false;

        Logger.recordOutput("Hood/Mechanism2d", mechanism);
        pivotLigament.setAngle(Math.toDegrees(hoodSimulation.getAngleRads()));
    }

    @Override
    public void setVoltage(double voltage) {
        hoodClosedLoop = false;
        hoodAppliedVoltage = MathUtility.clamp(voltage, -12.0, 12.0);
        hoodSimulation.setInputVoltage(hoodAppliedVoltage);
    }

    @Override
    public void setOL(double amperes) {
        setVoltage(hoodGearbox.getVoltage(
            hoodGearbox.getTorque(amperes),
            hoodSimulation.getVelocityRadPerSec()
        ));
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void setPosition(double position, double feedforward) {
        if (!hoodClosedLoop) {
            hoodControllerNeedsReset = true;
            hoodClosedLoop = true;
        }
        if (hoodControllerNeedsReset) {
            hoodPositionController.reset();
            hoodControllerNeedsReset = false;
        }

        setVoltage(hoodPositionController.calculate(hoodSimulation.getAngleRads(), position) + feedforward);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        hoodPositionController.setPID(kP, kI, kD);
    }

    @Override
    public void refreshData() {
        hoodSimulation.update(GlobalConstants.kLoopPeriodSeconds);
    }
}