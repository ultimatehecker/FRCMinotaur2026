package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSimulation implements IntakeIO {
    private final DCMotor rollerGearbox;
    private final DCMotorSim rollerSimulation;
    private double rollerAppliedVoltage = 0.0;
    private Double rollerVelocitySetpoint = null;

    private final DCMotor pivotGearbox;
    private final SingleJointedArmSim pivotSimulation;
    private double pivotAppliedVoltage = 0.0;
    private Double pivotPositionSetpoint  = null;

    private static final LoggedTunableNumber rollerKp  = new LoggedTunableNumber("Intake/Roller/kP", IntakeConstants.rollerSimulatedKp);
    private static final LoggedTunableNumber rollerKd  = new LoggedTunableNumber("Intake/Roller/kD", IntakeConstants.rollerSimulatedKd);
    private static final LoggedTunableNumber rollerKs  = new LoggedTunableNumber("Intake/Roller/kS", IntakeConstants.rollerSimulatedKs);
    private static final LoggedTunableNumber rollerKv  = new LoggedTunableNumber("Intake/Roller/kV", IntakeConstants.rollerSimulatedKv);
    private static final LoggedTunableNumber rollerKa  = new LoggedTunableNumber("Intake/Roller/kA", IntakeConstants.rollerSimulatedKa);

    private static final LoggedTunableNumber pivotKp   = new LoggedTunableNumber("Intake/Pivot/kP", IntakeConstants.pivotSimulatedKp);
    private static final LoggedTunableNumber pivotKd   = new LoggedTunableNumber("Intake/Pivot/kD", IntakeConstants.pivotSimulatedKd);
    private static final LoggedTunableNumber pivotKs   = new LoggedTunableNumber("Intake/Pivot/kS", IntakeConstants.pivotSimulatedKs);
    private static final LoggedTunableNumber pivotKv   = new LoggedTunableNumber("Intake/Pivot/kV", IntakeConstants.pivotSimulatedKv);
    private static final LoggedTunableNumber pivotKa   = new LoggedTunableNumber("Intake/Pivot/kA", IntakeConstants.pivotSimulatedKa);
    private static final LoggedTunableNumber pivotKCos = new LoggedTunableNumber("Intake/Pivot/kCos", IntakeConstants.pivotSimulatedKCos);

    private final ProfiledPIDController rollerController;
    private SimpleMotorFeedforward rollerFeedforward;

    private final ProfiledPIDController pivotController;
    private ArmFeedforward pivotFeedforward;

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(
        IntakeConstants.kIntakeLength.in(Meters) * 3,
        IntakeConstants.kIntakeLength.in(Meters) * 3
    );

    private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("Pivot Joint", IntakeConstants.kIntakeLength.in(Meters) * 1.5, IntakeConstants.kIntakeLength.in(Meters) * 1.5
    );

    private final LoggedMechanismLigament2d pivotLigament = mechanismRoot.append(
        new LoggedMechanismLigament2d(
            "Arm",
            IntakeConstants.kIntakeLength.in(Meters),
            Math.toDegrees(IntakeConstants.kIntakeStartingPosition.in(Radians)),
            4,
            new Color8Bit(Color.kOrange)
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

    public IntakeIOSimulation() {
        rollerGearbox = IntakeConstants.kRollerSimulatedGearbox;
        rollerSimulation = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                rollerGearbox,
                IntakeConstants.kRollerMOI.in(KilogramSquareMeters),
                IntakeConstants.kRollerMotorReduction
            ),
            rollerGearbox
        );

        rollerController = new ProfiledPIDController(rollerKp.get(), 0.0, rollerKd.get(),
            new TrapezoidProfile.Constraints(
                IntakeConstants.kRollerMaximumRotationalVelocity.in(RadiansPerSecond),
                IntakeConstants.kRollerMaximumRotationalAcceleration.in(RadiansPerSecondPerSecond)
            )
        );

        rollerFeedforward = new SimpleMotorFeedforward(rollerKs.get(), rollerKv.get(), rollerKa.get());

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

        pivotController = new ProfiledPIDController(pivotKp.get(), 0.0, pivotKd.get(),
            new TrapezoidProfile.Constraints(
                IntakeConstants.kPivotMaximumRotationalVelocity.in(RadiansPerSecond),
                IntakeConstants.kPivotMaximumRotationalAcceleration.in(RadiansPerSecondPerSecond)
            )
        );

        pivotFeedforward = new ArmFeedforward(
            pivotKs.get(), pivotKCos.get(), pivotKv.get(), pivotKa.get()
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            rollerVelocitySetpoint = null; // Might remove beacuse of transition between autonomous and teleop
            pivotPositionSetpoint  = null;

            setRollerVoltage(0.0);
            setPivotVoltage(0.0);
        }

        if (
            rollerKp.hasChanged(hashCode()) ||
            rollerKd.hasChanged(hashCode()) ||
            rollerKs.hasChanged(hashCode()) ||
            rollerKv.hasChanged(hashCode()) ||
            rollerKa.hasChanged(hashCode())
        ) {
            rollerController.setPID(rollerKp.get(), 0.0, rollerKd.get());
            rollerFeedforward = new SimpleMotorFeedforward(
                rollerKs.get(), rollerKv.get(), rollerKa.get()
            );
        }

        if (
            pivotKp.hasChanged(hashCode()) ||
            pivotKd.hasChanged(hashCode()) ||
            pivotKs.hasChanged(hashCode()) ||
            pivotKv.hasChanged(hashCode()) ||
            pivotKa.hasChanged(hashCode()) ||
            pivotKCos.hasChanged(hashCode())
        ) {
            pivotController.setPID(pivotKp.get(), 0.0, pivotKd.get());
            pivotFeedforward = new ArmFeedforward(
                pivotKs.get(), pivotKCos.get(), pivotKv.get(), pivotKa.get()
            );
        }

        if (rollerVelocitySetpoint != null) {
            double pidOutput = rollerController.calculate(rollerSimulation.getAngularVelocityRadPerSec(), rollerVelocitySetpoint);
            double ffOutput = rollerFeedforward.calculateWithVelocities(
                rollerController.getSetpoint().velocity,
                rollerController.getSetpoint().velocity
            );

            setRollerVoltage(pidOutput + ffOutput);
        }

        if (pivotPositionSetpoint != null) {
            double pidOutput = pivotController.calculate(pivotSimulation.getAngleRads(), pivotPositionSetpoint);
            double ffOutput = pivotFeedforward.calculateWithVelocities(
                pivotSimulation.getAngleRads(),
                pivotController.getSetpoint().velocity,
                pivotController.getSetpoint().velocity
            );

            setPivotVoltage(pidOutput + ffOutput);
        }

        rollerSimulation.update(0.02);
        pivotSimulation.update(0.02);

        inputs.rollerMotorConnected = true;
        inputs.rollerPosition = rollerSimulation.getAngularPositionRad();
        inputs.rollerVelocity = rollerSimulation.getAngularVelocityRadPerSec();
        inputs.rollerAcceleration = 0.0; 
        inputs.rollerAppliedVoltage = rollerAppliedVoltage;
        inputs.rollerSupplyCurrentAmperes = rollerSimulation.getCurrentDrawAmps();
        inputs.rollerTorqueCurrentAmperes = rollerGearbox.getCurrent(rollerSimulation.getAngularVelocityRadPerSec(), rollerAppliedVoltage);
        inputs.rollerTemperatureCelsius = 0.0; 

        inputs.pivotMotorConnected = true;
        inputs.pivotPosition = pivotSimulation.getAngleRads();
        inputs.pivotVelocity = pivotSimulation.getVelocityRadPerSec();
        inputs.pivotAcceleration = 0.0; 
        inputs.pivotAppliedVoltage = pivotAppliedVoltage;
        inputs.pivotSupplyCurrentAmperes = pivotSimulation.getCurrentDrawAmps();
        inputs.pivotMotorTempuratureCelcius = 0.0; 

        pivotLigament.setAngle(Math.toDegrees(pivotSimulation.getAngleRads()));
        Logger.recordOutput("Intake/Mechanism2d", mechanism);
        
        updateRollerColor();
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        rollerSimulation.setInputVoltage(rollerAppliedVoltage);
    }

    @Override
    public void setRollerTorqueCurrent(double amperes) {
        setRollerVoltage(rollerGearbox.getVoltage(
            rollerGearbox.getTorque(amperes),
            rollerSimulation.getAngularVelocityRadPerSec()
        ));
    }

     @Override
    public void setRollerVelocity(double velocity) {
        rollerVelocitySetpoint = velocity;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        pivotSimulation.setInputVoltage(pivotAppliedVoltage);
    }

    @Override
    public void setPivotPosition(double position) {
        pivotPositionSetpoint = position;
    }

    @Override
    public void stopRollers() {
        setRollerVoltage(0.0);
    }

    private void updateRollerColor() {
        double velocity = rollerSimulation.getAngularVelocityRadPerSec();

        if (Math.abs(velocity) < IntakeConstants.kRollerIdleThreshold.in(RadiansPerSecond)) {
            rollerLigament.setColor(new Color8Bit(Color.kYellow));
        } else if (velocity > 0) {
            rollerLigament.setColor(new Color8Bit(Color.kGreen));
        } else {
            rollerLigament.setColor(new Color8Bit(Color.kRed));
        }
    }
}