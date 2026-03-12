package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeIOSimulation implements IntakeIO {
    private final DCMotor rollerGearbox;
    private final DCMotorSim rollerSimulation;
    private double rollerAppliedVoltage = 0.0;

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
    public void updateInputs(IntakeIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            pivotControllerNeedsReset = true;
        }

        rollerSimulation.update(GlobalConstants.kLoopPeriodSeconds);
        pivotSimulation.update(GlobalConstants.kLoopPeriodSeconds);

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

        Logger.recordOutput("Intake/Mechanism2d", mechanism);

        pivotLigament.setAngle(Math.toDegrees(pivotSimulation.getAngleRads()));
        
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
    public void setPivotVoltage(double voltage) {
        pivotClosedLoop = false;
        pivotAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        pivotSimulation.setInputVoltage(pivotAppliedVoltage);
    }

    @Override
    public void setPivotPosition(double position, double feedforward) {
        if (!pivotClosedLoop) {
            pivotControllerNeedsReset = true;
            pivotClosedLoop = true;
        }
        if (pivotControllerNeedsReset) {
            pivotController.reset();
            pivotControllerNeedsReset = false;
        }

        setPivotVoltage(pivotController.calculate(pivotSimulation.getAngleRads(), position) + feedforward);
    }

    @Override
    public void stopRollers() {
        setRollerVoltage(0.0);
    }

    @Override
    public void setPivotPID(double kP, double kI, double kD) {
        pivotController.setPID(kP, kI, kD);
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

    @Override
    public void refreshData() {
    
    }
}