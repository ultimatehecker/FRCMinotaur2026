package frc.robot.subsystems.intake.slam;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.math.EqualsUtility;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

public class Slam {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Slam/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Slam/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Slam/kS");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Slam/kG");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Slam/kA");

    private static final LoggedTunableNumber kMaximumVelocityMetersPerSec = new LoggedTunableNumber("Slam/MaximumVelocityMetersPerSec", 8.0);
    private static final LoggedTunableNumber kMaximumAccelerationMetersPerSec2 = new LoggedTunableNumber("Slam/MaximumAccelerationMetersPerSec2", 80.0);
    private static final LoggedTunableNumber kHomingVoltage = new LoggedTunableNumber("Slam/HomingVoltage", -3.0);
    private static final LoggedTunableNumber kHomingTimeSeconds = new LoggedTunableNumber("Slam/HomingTimeSeconds", 0.4);
    private static final LoggedTunableNumber kHomingVelocityThreshold = new LoggedTunableNumber("Slam/HomingVelocityThreshold", 0.1);

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(IntakeConstants.pivotkP);
                kD.initDefault(IntakeConstants.pivotkD);
                kS.initDefault(IntakeConstants.pivotkS);
                kG.initDefault(IntakeConstants.pivotkG);
                kA.initDefault(IntakeConstants.pivotkA);
            }
            case SIMBOT -> {
                kP.initDefault(10.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kG.initDefault(0.0);
                kA.initDefault(0.0);
            }
        }
    }

    private final SlamIO io;
    private final SlamIOInputsAutoLogged inputs = new SlamIOInputsAutoLogged();

    // Connected debouncer
    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Alert motorDisconnectedAlert = new Alert("Slam motor disconnected!", Alert.AlertType.kError);
    private final Alert motorTemperatureAlert = new Alert("Slam motor is overheating!", Alert.AlertType.kWarning);

    @AutoLogOutput(key = "Intake/Slam/BrakeModeEnabled")
    private boolean brakeModeEnabled = true;

    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    @Getter private double requestedPosition = IntakeConstants.kIntakeMinimumPosition.in(Radians);
    private boolean stopProfile = false;

    @AutoLogOutput(key = "Intake/Slam/HomedPositionRadians")
    private double homedPosition = 0.0;

    @AutoLogOutput(key = "Intake/Slam/Homed")
    @Getter
    private boolean homed = true; // TODO: Revert to false when the homing system works

    private Debouncer homingDebouncer = new Debouncer(kHomingTimeSeconds.get());
    private final Command homingCommand;

    @Getter
    @Accessors(fluent = true)
    @AutoLogOutput(key = "Intake/Slam/Profile/AtGoal")
    private boolean atGoal = false;

    @Getter
    @Accessors(fluent = true)
    private boolean wantsToDeploy = false;

    public Slam(SlamIO io) {
        this.io = io;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (inputs) {
                io.updateInputs(inputs);
            }
        },
        io);

        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(kMaximumVelocityMetersPerSec.get(), kMaximumAccelerationMetersPerSec2.get())
        );

        homingCommand = homingSequence();
    }

    public void periodic() {
        synchronized (inputs) {
            Logger.processInputs("Intake/Slam", inputs);
        }

        motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isMotorConnected) && !Robot.isJITing());
        motorTemperatureAlert.set(inputs.temperatureFault);

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get());
        }

        if (kMaximumVelocityMetersPerSec.hasChanged(hashCode()) || kMaximumAccelerationMetersPerSec2.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(kMaximumVelocityMetersPerSec.get(), kMaximumAccelerationMetersPerSec2.get())
            );
        }

        // Tell intake to deploy or not when disabled
        wantsToDeploy = !homed || (getMeasuredAngleRad() < IntakeConstants.kIntakeMaximumPosition.in(Radians) / 2.0);

        if (DriverStation.isEnabled() && !homed && !homingCommand.isScheduled()) {
            homingCommand.schedule();
        }

        // Run profile
        final boolean shouldRunProfile =
            !stopProfile
            && brakeModeEnabled
            && (homed || GlobalConstants.getRobot() == GlobalConstants.RobotType.SIMBOT)
            && DriverStation.isEnabled();

        Logger.recordOutput("Intake/Slam/RunningProfile", shouldRunProfile);

        if (shouldRunProfile) {
            var goalState = new State(MathUtil.clamp(requestedPosition, 0.0, IntakeConstants.kIntakeMaximumPosition.in(Radians)), 0.0);
            double previousVelocity = setpoint.velocity;
            setpoint = profile.calculate(GlobalConstants.kLoopPeriodSeconds, setpoint, goalState);

            if (setpoint.position < 0.0 || setpoint.position > IntakeConstants.kIntakeMaximumPosition.in(Radians)) {
                setpoint = new State(MathUtil.clamp(setpoint.position, 0.0, IntakeConstants.kIntakeMaximumPosition.in(Radians)), 0.0);
            }

            // Check at goal
            atGoal = EqualsUtility.epsilonEquals(setpoint.position, goalState.position) && EqualsUtility.epsilonEquals(setpoint.velocity, goalState.velocity);
            // Run
            double acceleration = (setpoint.velocity - previousVelocity) / GlobalConstants.kLoopPeriodSeconds;
            io.setPosition(
                setpoint.position + homedPosition,
                kS.get() * Math.signum(setpoint.velocity) + kG.get() * Math.cos(setpoint.position) + kA.get() * acceleration
            );

            Logger.recordOutput("Intake/Slam/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("Intake/Slam/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("Intake/Slam/Profile/GoalPositionMeters", goalState.position);
            Logger.recordOutput("Intake/Slam/Profile/GoalVelocityMetersPerSec", goalState.velocity);
        } else {
            // Reset setpoint
            setpoint = new State(getMeasuredAngleRad(), 0.0);

            // Clear logs
            Logger.recordOutput("Intake/Slam/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("Intake/Slam/Profile/SetpointVelocityMetersPerSec", 0.0);
            Logger.recordOutput("Intake/Slam/Profile/GoalPositionMeters", 0.0);
            Logger.recordOutput("Intake/Slam/Profile/GoalVelocityMetersPerSec", 0.0);
        }

        // Log state
        Logger.recordOutput("Intake/Slam/MeasuredVelocityMetersPerSec", inputs.velocityRadiansPerSecond);
        LoggedTracer.record("Slam");
    }

    public void setSetpointPosition(double requestedPosition) {
        if (requestedPosition == this.requestedPosition) return;
        this.requestedPosition = requestedPosition;
        atGoal = false;
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    /** Set current position of slam to home. */
    public void setHome() {
        homedPosition = inputs.positionRadians;
        homed = true;
    }

    public Command homingSequence() {
        return Commands.startRun(() -> {
            stopProfile = true;
            homed = false;
            homingDebouncer = new Debouncer(kHomingTimeSeconds.get());
            homingDebouncer.calculate(false);
        }, () -> {
            if (!brakeModeEnabled) return;
            io.setVoltage(kHomingVoltage.get());
            homed = homingDebouncer.calculate(Math.abs(inputs.velocityRadiansPerSecond) <= kHomingVelocityThreshold.get() && Math.abs(inputs.appliedVoltage) >= kHomingVoltage.get() * 0.7);
        })
        .until(() -> homed)
        .andThen(this::setHome)
        .finallyDo(() -> {
            stopProfile = false;
        });
    }

    public void overrideHoming() {
        homed = false;
    }

    /** Get position of slam with maxAngle at home */
    @AutoLogOutput(key = "Intake/Slam/MeasuredAngleRads")
    public double getMeasuredAngleRad() {
        return inputs.positionRadians - homedPosition;
    }
}