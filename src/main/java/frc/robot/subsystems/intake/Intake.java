package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.slam.Slam;
import frc.robot.subsystems.intake.slam.SlamIO;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;

import lombok.Getter;

public class Intake extends SubsystemBase {
    private static final LoggedTunableNumber kStowedPosition = new LoggedTunableNumber("Intake/Slam/StowedPosition", 56.9);
    private static final LoggedTunableNumber kDeployedPosition = new LoggedTunableNumber("Intake/Slam/DeployedPosition", 187.4);
    private static final LoggedTunableNumber kHalfDeployPosition = new LoggedTunableNumber("Intake/Slam/HalfPosition", 110.0);
    private static final LoggedTunableNumber kFeedPosition = new LoggedTunableNumber("Intake/Slam/FeedPosition", 140.0);
    private static final LoggedTunableNumber kRollerVoltage = new LoggedTunableNumber("Intake/Roller/IntakeVoltage", 7.0);
    private static final LoggedTunableNumber kExhaustVoltage = new LoggedTunableNumber("Intake/Roller/ExhaustVoltage", -6.0);
    private static final LoggedTunableNumber kIdleVoltage = new LoggedTunableNumber("Intake/Roller/IdleVoltage", 4.0);

    public enum IntakeGoal {
        IDLE,
        STOW,
        DEPLOY,
        DEPLOY_HALF,
        FEED,
        INTAKE,
        EXHAUST
    }

    public enum IntakeState {
        IDLE,
        STOWING,
        STOWED,
        DEPLOYING,
        DEPLOYED,
        DEPLOYED_HALF,
        FEED,
        INTAKING,
        EXHAUSTING
    }

    private final RollerSystem roller;
    private final Slam slam;

    @Getter private IntakeGoal goal = IntakeGoal.IDLE;
    @Getter private IntakeState state = IntakeState.IDLE;

    public Intake(SlamIO slamIO, RollerSystemIO rollerIO) {
        slam = new Slam(slamIO);
        roller = new RollerSystem("Intake Roller", "Intake/Roller", rollerIO);
    }

    @Override
    public void periodic() {
        slam.periodic();
        roller.periodic();

        state = evaluateState();
        applyOutputs();

        Logger.recordOutput("Intake/Goal", goal.toString());
        Logger.recordOutput("Intake/State", state.toString());
        LoggedTracer.record("IntakePeriodic");
    }

    private IntakeState evaluateState() {
        return switch (goal) {
            case IDLE -> IntakeState.IDLE;
            case STOW -> {
                slam.setSetpointPosition(Units.degreesToRadians(kStowedPosition.get()));
                yield slam.atGoal() ? IntakeState.STOWED : IntakeState.STOWING;
            }
            case DEPLOY -> {
                slam.setSetpointPosition(Units.degreesToRadians(kDeployedPosition.get()));
                yield slam.atGoal() ? IntakeState.DEPLOYED : IntakeState.DEPLOYING;
            }
            case DEPLOY_HALF -> {
                slam.setSetpointPosition(Units.degreesToRadians(kHalfDeployPosition.get()));
                yield slam.atGoal() ? IntakeState.DEPLOYED_HALF : IntakeState.DEPLOYING;
            }
            case FEED -> {
                slam.setSetpointPosition(Units.degreesToRadians(kFeedPosition.get()));
                yield slam.atGoal() ? IntakeState.FEED : IntakeState.DEPLOYING;
            }
            case INTAKE -> IntakeState.INTAKING;
            case EXHAUST -> IntakeState.EXHAUSTING;
        };
    }

    private void applyOutputs() {
        switch (state) {
            case IDLE -> {
                roller.stop();
            }

            case STOWING -> {
                slam.setSetpointPosition(Units.degreesToRadians(kStowedPosition.get()));
                roller.stop();
            }

            case STOWED -> {
                roller.stop();
            }

            case DEPLOYING -> {
                double target = goal == IntakeGoal.DEPLOY_HALF ? kHalfDeployPosition.get() : kDeployedPosition.get();
                slam.setSetpointPosition(Units.degreesToRadians(target));
                roller.setVoltage(kRollerVoltage.get());
            }

            case DEPLOYED, DEPLOYED_HALF -> {
                roller.setVoltage(kIdleVoltage.get());
            }

            case FEED -> {
                roller.setVoltage(kIdleVoltage.get());
            }

            case INTAKING -> {
                slam.setSetpointPosition(Units.degreesToRadians(kDeployedPosition.get()));
                roller.setVoltage(kRollerVoltage.get());
            }

            case EXHAUSTING -> {
                slam.setSetpointPosition(Units.degreesToRadians(kDeployedPosition.get()));
                roller.setVoltage(kExhaustVoltage.get());
            }
        }
    }

    public void setGoal(IntakeGoal goal) {
        this.goal = goal;
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        slam.setBrakeMode(enabled.getAsBoolean());
        roller.setBrakeMode(enabled.getAsBoolean());
    }
}
