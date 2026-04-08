package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.robot.subsystems.intake.slam.Slam;
import frc.robot.subsystems.intake.slam.SlamIO;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Intake extends SubsystemBase {
    private static final LoggedTunableNumber slamGoalDebounceTime = new LoggedTunableNumber("Intake/Slam/DebounceTime", 0.5);
    private Debouncer slamGoalDebouncer = new Debouncer(slamGoalDebounceTime.get(), DebounceType.kRising);

    @RequiredArgsConstructor
    public enum RollerGoal {
        STOP(new LoggedTunableNumber("Intake/Roller/StopVoltage", 0.0)),
        INTAKE(new LoggedTunableNumber("Intake/Roller/IntakeVoltage", 12.0)),
        EXHAUST(new LoggedTunableNumber("Intake/Roller/ExhaustVoltage", -6.0)),
        IDLE(new LoggedTunableNumber("Intake/Roller/IdleVoltage", 1.5));

        private final DoubleSupplier voltage;

        public double getVoltage() {
            return voltage.getAsDouble();
        }
    }

    @RequiredArgsConstructor
    public enum SlamGoal {
        RETRACT(new LoggedTunableNumber("Intake/Slam/RetractPositionDegrees", 59.6)),
        DEPLOY(new LoggedTunableNumber("Intake/Slam/DeployPositionDegrees", 187.4)),
        FEED(new LoggedTunableNumber("Intake/Slam/FeedPositionDegrees", 120.0)),
        CLIMB(new LoggedTunableNumber("Intake/Slam/ClimbPositionDegrees", 70.0));

        private final DoubleSupplier positionDegrees;

        public double getPositionRadians() {
            return Units.degreesToRadians(positionDegrees.getAsDouble());
        }
    }

    public enum IntakeState {
        DEPLOYED,
        RETRACTED,
        FEED,
        CLIMB,
        MOVING
    }

    private final RollerSystem roller;
    private final Slam slam;

    @Getter @Setter @AutoLogOutput private RollerGoal intakeGoal = RollerGoal.STOP;
    @Getter @Setter @AutoLogOutput private SlamGoal slamGoal = SlamGoal.RETRACT;
    @Getter @AutoLogOutput private IntakeState slamState = IntakeState.RETRACTED;

    public Intake(SlamIO slamIO, RollerSystemIO rollerIO) {
        slam = new Slam(slamIO);
        roller = new RollerSystem("Intake Roller", "Intake/Roller", rollerIO);
    }

    @Override
    public void periodic() {
        slam.periodic();
        roller.periodic();

        if (slamGoalDebounceTime.hasChanged(hashCode())) {
            slamGoalDebouncer = new Debouncer(slamGoalDebounceTime.get());
        } 

        double rollerVoltage = 0.0;
        switch (intakeGoal) {
            case STOP -> {
                rollerVoltage = 0.0;
            }

            case INTAKE, EXHAUST, IDLE -> {
                rollerVoltage = intakeGoal.getVoltage();
            }
        }

        roller.setVoltage(rollerVoltage);

        switch (slamGoal) {
            case DEPLOY -> {
                slam.setSetpointPosition(slamGoal.getPositionRadians());
                slamState = slamGoalDebouncer.calculate(slam.atGoal()) ? IntakeState.DEPLOYED : IntakeState.MOVING;
            }

            case RETRACT -> {
                slam.setSetpointPosition(slamGoal.getPositionRadians());
                slamState = slamGoalDebouncer.calculate(slam.atGoal()) ? IntakeState.RETRACTED : IntakeState.MOVING;
            }

            case FEED -> {
                slam.setSetpointPosition(slamGoal.getPositionRadians());
                slamState = slamGoalDebouncer.calculate(slam.atGoal()) ? IntakeState.FEED : IntakeState.MOVING;
            }

            case CLIMB -> {
                slam.setSetpointPosition(slamGoal.getPositionRadians());
                slamState = slamGoalDebouncer.calculate(slam.atGoal()) ? IntakeState.CLIMB : IntakeState.MOVING;
            }
        }

        LoggedTracer.record("Intake/Periodic");
    }

    public void setBrakeMode(BooleanSupplier brakeModeEnabled) {
        slam.setBrakeMode(brakeModeEnabled.getAsBoolean());
        roller.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public boolean isHomed() {
        return slam.isHomed();
    }

    public Command homeSlam() {
        return slam.homingSequence();
    }

}
