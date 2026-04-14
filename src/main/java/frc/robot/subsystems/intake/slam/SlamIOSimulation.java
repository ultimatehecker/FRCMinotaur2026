package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.minolib.math.MathUtility;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;

public class SlamIOSimulation implements SlamIO {
    private final DCMotor pivotGearbox;
    private final SingleJointedArmSim pivotSimulation;
    private double pivotAppliedVoltage = 0.0;

    private final PIDController pivotController = new PIDController(0.0, 0.0, 0.0);
    private boolean pivotClosedLoop = false;
    private boolean pivotControllerNeedsReset = false;

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(
        IntakeConstants.kIntakeLength.in(Meters) * 3,
        IntakeConstants.kIntakeLength.in(Meters) * 3
    );

    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("Pivot Joint", IntakeConstants.kIntakeLength.in(Meters) * 1.5, IntakeConstants.kIntakeLength.in(Meters) * 1.5);

    private final LoggedMechanismLigament2d pivotLigament = mechanismRoot.append(
        new LoggedMechanismLigament2d(
            "Arm",
            IntakeConstants.kIntakeLength.in(Meters),
            IntakeConstants.kIntakeMinimumPosition.in(Degrees),
            4,
            new Color8Bit(Color.kMaroon)
        )
    );

    private final LoggedMechanismLigament2d rollerLigament = pivotLigament.append(
        new LoggedMechanismLigament2d(
            "Roller",
            IntakeConstants.kIntakeLength.in(Meters) * 0.15,
            90.0, 
            6,
            new Color8Bit(Color.kYellow) 
        )
    );

    public SlamIOSimulation() {
        pivotGearbox = IntakeConstants.kPivotSimulatedGearbox;
        pivotSimulation = new SingleJointedArmSim(
            pivotGearbox,
            IntakeConstants.kPivotMotorReduction,
            IntakeConstants.kPivotMOI.in(KilogramSquareMeters),
            IntakeConstants.kIntakeLength.in(Meters),
            IntakeConstants.kIntakeMinimumPosition.in(Radians),
            IntakeConstants.kIntakeMaximumPosition.in(Radians),
            true, 
            IntakeConstants.kIntakeStartingPosition.in(Radians)
        );
    }

    @Override
    public void updateInputs(SlamIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            pivotControllerNeedsReset = true;
        }

        inputs.isMotorConnected = true;
        inputs.positionRadians = pivotSimulation.getAngleRads();
        inputs.velocityRadiansPerSecond = pivotSimulation.getVelocityRadPerSec();
        inputs.appliedVoltage = pivotAppliedVoltage;
        inputs.torqueCurrentAmperes = pivotGearbox.getCurrent(pivotSimulation.getVelocityRadPerSec(), pivotAppliedVoltage);
        inputs.supplyCurrentAmperes = pivotSimulation.getCurrentDrawAmps();
        inputs.temperatureCelsius = 0.0; 
        inputs.temperatureFault = false;

        Logger.recordOutput("Intake/Mechanism2d", mechanism);
        pivotLigament.setAngle(Math.toDegrees(pivotSimulation.getAngleRads()));
    }

    @Override
    public void setVoltage(double voltage) {
        pivotAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        pivotSimulation.setInputVoltage(pivotAppliedVoltage);
    }

    @Override
    public void setOL(double amperes) {
        setVoltage(pivotGearbox.getVoltage(
            pivotGearbox.getTorque(amperes),
            pivotSimulation.getVelocityRadPerSec()
        ));
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void setPosition(double position, double feedforward) {
        if (!pivotClosedLoop) {
            pivotControllerNeedsReset = true;
            pivotClosedLoop = true;
        }
        if (pivotControllerNeedsReset) {
            pivotController.reset();
            pivotControllerNeedsReset = false;
        }

        setVoltage(pivotController.calculate(pivotSimulation.getAngleRads(), position) + feedforward);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        pivotController.setPID(kP, kI, kD);
    }

    @Override
    public void refreshData() {
        pivotSimulation.update(GlobalConstants.kLoopPeriodSeconds);
    }
}
