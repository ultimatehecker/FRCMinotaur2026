package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.HoodConstants;

import lombok.Getter;

public class Hood extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA");

    private static final LoggedTunableNumber kMinimumAngleDegrees = new LoggedTunableNumber("Hood/MinimumAngle", HoodConstants.kHoodMinimumPosition.in(Degrees));
    private static final LoggedTunableNumber kMaximumAngleDegrees = new LoggedTunableNumber("Hood/MaximumAngle", HoodConstants.kHoodMaximumPosition.in(Degrees));

    private static final LoggedTunableNumber toleranceDegrees = new LoggedTunableNumber("Hood/ToleranceDegrees", 2.0);
    private static final LoggedTunableNumber readyDebounceSeconds = new LoggedTunableNumber("Hood/ReadyDebounceSeconds", 0.08);

    public enum HoodGoal {
        IDLE,
        STOW,
        AIM,
        HOME
    }

    public enum HoodState {
        IDLE,
        STOWING,
        MOVING_TO_ANGLE,
        AT_ANGLE,
        HOMING
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(HoodConstants.kP);
                kD.initDefault(HoodConstants.kD);
                kS.initDefault(HoodConstants.kD);
                kV.initDefault(HoodConstants.kV);
                kG.initDefault(HoodConstants.kG);
                kA.initDefault(HoodConstants.kA);
            }

            case SIMBOT -> {
                kP.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kG.initDefault(0.0);
                kA.initDefault(0.0);
                kP.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kG.initDefault(0.0);
                kA.initDefault(0.0);
            }
        }
    }

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert motorDisconnectedAlert = new Alert("The hood motor is disconnected!", AlertType.kError);
    private final Alert motorOverheatingAlert = new Alert("The hood motor is overheating!", AlertType.kWarning);

    @Getter private HoodGoal goal = HoodGoal.IDLE;
    @Getter private HoodState state = HoodState.IDLE;
    @Getter private double targetDegrees = 0.0;

    private final Debouncer readyDebouncer = new Debouncer(readyDebounceSeconds.get(), Debouncer.DebounceType.kFalling);

    public Hood(HoodIO io) {
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
            Logger.processInputs("Hood", inputs);
        }
    
        motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isMotorConnected) && !Robot.isJITing());
        motorOverheatingAlert.set(inputs.temperatureFault);
        motorOverheatingAlert.set(inputs.temperatureFault);

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get());
        }

        if (readyDebounceSeconds.hasChanged(hashCode())) {
            readyDebouncer.setDebounceTime(readyDebounceSeconds.get());
        }

        state = handleStateTransition();
        applyState();

        Logger.recordOutput("Hood/Goal", goal.toString());
        Logger.recordOutput("Hood/State", state.toString());
        
        LoggedTracer.record("HoodPeriodic");
    }

    private HoodState handleStateTransition() {
        return switch (goal) {
            case IDLE -> HoodState.IDLE;
            case STOW -> atTarget(kMinimumAngleDegrees.get()) ? HoodState.AT_ANGLE : HoodState.STOWING;
            case AIM -> atTarget(targetDegrees) ? HoodState.AT_ANGLE : HoodState.MOVING_TO_ANGLE;
            case HOME -> HoodState.HOMING;
        };
    }

    private void applyState() {
        switch (state) {    
            case IDLE -> io.stop();
            case STOWING -> {
                io.setPosition(
                    MathUtil.clamp(
                        Units.degreesToRadians(kMinimumAngleDegrees.get()), 
                        Units.degreesToRadians(kMinimumAngleDegrees.get()), 
                        Units.degreesToRadians(kMaximumAngleDegrees.get())
                    ), 
                    0.0
                );
            }

            case MOVING_TO_ANGLE, AT_ANGLE ->  {
                io.setPosition(
                    MathUtil.clamp(
                        Units.degreesToRadians(targetDegrees), 
                        Units.degreesToRadians(kMinimumAngleDegrees.get()), 
                        Units.degreesToRadians(kMaximumAngleDegrees.get())
                    ), 
                    0.0
                );
            }
            case HOMING -> {

            }
        }
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        io.setBrakeMode(enabled.getAsBoolean());
    }

    public void setGoal(HoodGoal goal) {
        this.goal = goal;
    }

    public void setAngle(double angleDegrees) {
        this.goal = HoodGoal.AIM;
        this.targetDegrees = angleDegrees;
    }

    public void stow() {
        this.goal = HoodGoal.STOW;
    }

    public void stop() {
        this.goal = HoodGoal.IDLE;
        this.targetDegrees = 0.0;

        readyDebouncer.calculate(false);
    }

    private boolean atTarget(double targetDegrees) {
        return Math.abs(inputs.positionRadians - Units.degreesToRadians(targetDegrees)) <= Units.degreesToRadians(toleranceDegrees.get());
    }

    public boolean atSetpoint() {
        return state == HoodState.AT_ANGLE;
    }

    public boolean isReady() {
        return readyDebouncer.calculate(atSetpoint());
    }
}
