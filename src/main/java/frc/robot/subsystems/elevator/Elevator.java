package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.Robot;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.shooter.hood.Hood.HoodState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Elevator extends SubsystemBase {
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kV");
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

    private static final LoggedTunableNumber toleranceDegrees = new LoggedTunableNumber("Elevator/ToleranceRadians", 0.0524);
    private static final LoggedTunableNumber readyDebounceSeconds = new LoggedTunableNumber("Elevator/ReadyDebounceSeconds", 0.08);

    private static final LoggedTunableNumber kStowedPosition = new LoggedTunableNumber("Elevator/StowedPosition", 0.0);
    private static final LoggedTunableNumber kDeployedPosition = new LoggedTunableNumber("Elevator/DeployedPosition", 13.0);
    private static final LoggedTunableNumber kClimbedPosition = new LoggedTunableNumber("Elevator/ClimbPosition", 6.0);

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kG.initDefault(0.0);
                kA.initDefault(0.0);
            }
            case SIMBOT -> {
                kP.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kG.initDefault(0.0);
                kA.initDefault(0.0);
            }
        }
    }

    public enum ElevatorGoal {
        IDLE,
        STOW,
        DEPLOY,
        CLIMB,
        HOME,
        MANUAL
    }

    public enum ElevatorState {
        IDLE,
        STOWING,
        STOWED,
        DEPLOYING,
        DEPLOYED,
        CLIMBING,
        CLIMBED,
        HOMING,
        MANUAL
    }

    @Getter @Setter @AutoLogOutput private ElevatorGoal goal = ElevatorGoal.IDLE;
    @Getter @AutoLogOutput private ElevatorState state = ElevatorState.IDLE;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private Debouncer readyDebouncer = new Debouncer(readyDebounceSeconds.get(), Debouncer.DebounceType.kFalling);

    private final Alert motorDisconnectedAlert = new Alert("The elevator motor is disconnected!", AlertType.kError);
    private final Alert motorTemperatureAlert = new Alert("The elevator motor is overheating!", AlertType.kWarning);

    private double voltageSetpoint = 0.0;

    public Elevator(ElevatorIO io) {
        this.io = io;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (inputs) {
                io.updateInputs(inputs);
            }
        },
        io);
    }

    @Override
    public void periodic() {
        synchronized (inputs) {
            Logger.processInputs("Elevator", inputs);
        }

        motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isMotorConnected) && !Robot.isJITing());
        motorTemperatureAlert.set(inputs.temperatureFault);

        if (readyDebounceSeconds.hasChanged(hashCode())) {
            readyDebouncer = new Debouncer(readyDebounceSeconds.get());
        }

        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get());
        }

        state = handleStateTransition();
        applyState();

        Logger.recordOutput("Elevator/Goal", goal.toString());
        Logger.recordOutput("Elevator/State", state.toString());
        LoggedTracer.record("ElevatorPeriodic");
    }

    public ElevatorState handleStateTransition() {
        return switch (goal) {
            case IDLE -> ElevatorState.IDLE;
            case STOW -> { 
                io.setPosition(
                    kStowedPosition.get(), 
                    kS.get() * Math.signum(ElevatorConstants.kMaximumRotationalVelocity.in(RadiansPerSecond)) + kG.get()
                );

                yield atSetpoint() ? ElevatorState.STOWED : ElevatorState.STOWING;
            }
            case DEPLOY -> {
                io.setPosition(
                    kDeployedPosition.get(), 
                    kS.get() * Math.signum(ElevatorConstants.kMaximumRotationalVelocity.in(RadiansPerSecond)) + kG.get()
                );

                yield atSetpoint() ? ElevatorState.DEPLOYED : ElevatorState.DEPLOYING;
            }
            case CLIMB -> {
                io.setPosition(
                    kClimbedPosition.get(), 
                    kS.get() * Math.signum(ElevatorConstants.kMaximumRotationalVelocity.in(RadiansPerSecond)) + kG.get()
                );

                yield atSetpoint() ? ElevatorState.CLIMBED : ElevatorState.CLIMBING;
            }
            case HOME -> ElevatorState.HOMING;
            case MANUAL -> ElevatorState.MANUAL;
        };
    }

    public void applyState() {
        switch (state) {
            case IDLE -> io.stop();
            case STOWING -> { 
                io.setPosition(
                    kStowedPosition.get(), 
                    kS.get() * Math.signum(ElevatorConstants.kMaximumRotationalVelocity.in(RadiansPerSecond)) + kG.get()
                );
            }
            case STOWED -> io.stop();
            case DEPLOYING -> {
                io.setPosition(
                    kDeployedPosition.get(), 
                    kS.get() * Math.signum(ElevatorConstants.kMaximumRotationalVelocity.in(RadiansPerSecond)) + kG.get()
                );
            }
            case DEPLOYED -> io.stop();
            case CLIMBING -> {
                io.setPosition(
                    kClimbedPosition.get(), 
                    kS.get() * Math.signum(ElevatorConstants.kMaximumRotationalVelocity.in(RadiansPerSecond)) + kG.get()
                );
            }
            case CLIMBED -> io.stop();
            case HOMING -> {

            }
            case MANUAL -> io.setVoltage(voltageSetpoint);
        }
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        io.setBrakeMode(enabled.getAsBoolean());
    }

    public void runVoltage(double voltageSetpoint) {
        goal = ElevatorGoal.MANUAL;
        this.voltageSetpoint = voltageSetpoint;
    }

    private boolean atTarget(double targetDegrees) {
        return Math.abs(inputs.positionRadians - Units.degreesToRadians(targetDegrees)) <= Units.degreesToRadians(toleranceDegrees.get());
    }

    public boolean atSetpoint() {
        return state == ElevatorState.CLIMBED || state == ElevatorState.DEPLOYED || state == ElevatorState.STOWED;
    }

    public boolean isReady() {
        return readyDebouncer.calculate(atSetpoint());
    }
}
