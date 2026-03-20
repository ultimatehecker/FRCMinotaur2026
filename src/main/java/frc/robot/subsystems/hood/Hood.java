package frc.robot.subsystems.hood;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.math.EqualsUtility;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.intake.Intake.PivotGoal;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

public class Hood extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA");

    private static final LoggedTunableNumber kMaximumAngle = new LoggedTunableNumber("Hood/MaximumAngle", HoodConstants.kHoodMaximumPosition.in(Radians));
    private static final LoggedTunableNumber kMaximumVelocityRadiansPerSecond = new LoggedTunableNumber("Hood/MaxVelocityRadiansPerSecond", HoodConstants.kHoodMaximumRotationalVelocity.in(RadiansPerSecond));
    private static final LoggedTunableNumber kMaximumAccelerationRadiansPerSecond2 = new LoggedTunableNumber("Hood/MaxAccelerationRadiansPerSecond2", HoodConstants.kHoodMaximumRotationalAcceleration.in(RadiansPerSecondPerSecond));
    private static final LoggedTunableNumber kHomingVolts = new LoggedTunableNumber("Hood/HomingVoltage", -3.0);
    private static final LoggedTunableNumber kHomingTimeoutSeconds = new LoggedTunableNumber("Hood/HomingTimeSeconds", 0.4);
    private static final LoggedTunableNumber kHomingVelocityThreshold = new LoggedTunableNumber("Hood/HomingVelocityThreshold", 0.1);

    @RequiredArgsConstructor
    public enum HoodGoal {
        MINIMUM(new LoggedTunableNumber("Hood/FullyDownPosition", 10)),
        MINIMUM_MIDPOINT(new LoggedTunableNumber("Hood/FullyDownPosition", 17.75)),
        MIDPOINT(new LoggedTunableNumber("Hood/HalwayPosition", 24.5)),
        MAXIMUM(new LoggedTunableNumber("Hood/FullyUpPosition", 39.0));

        private final DoubleSupplier positionDegrees;

        public double getAngleRadians() {
            return Units.degreesToRadians(positionDegrees.getAsDouble());
        }
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
                kP.initDefault(HoodConstants.simulatedKp);
                kD.initDefault(HoodConstants.simulatedKd);
                kS.initDefault(HoodConstants.simulatedKs);
                kV.initDefault(HoodConstants.simulatedKv);
                kG.initDefault(HoodConstants.simulatedKg);
                kA.initDefault(HoodConstants.simulatedKa);
            }
        }
    }

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert motorDisconnectedAlert = new Alert("The hood motor is disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Hood/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> true;

    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    @Getter private HoodGoal goal = HoodGoal.MINIMUM;
    private boolean stopProfile = false;

    @AutoLogOutput(key = "Hood/HomedPositionRadians")
    private double homedPosition = 0.0;

    @AutoLogOutput(key = "Hood/Homed")
    @Getter
    private boolean isHomed = true;

    private Debouncer homingDebouncer = new Debouncer(kHomingTimeoutSeconds.get());
    private final Command homingCommand;

    @Getter
    @Accessors(fluent = true)
    @AutoLogOutput(key = "Hood/Profile/AtGoal")
    private boolean atGoal = false;

    @Getter
    @Accessors(fluent = true)
    private boolean wantsToDeploy = false;

    public Hood(HoodIO io) {
        this.io = io;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (inputs) {
                io.updateInputs(inputs);
            }
        },
        io);

        homingCommand = homingSequence();
    }

    @Override
    public void periodic() {
        synchronized (inputs) {
            Logger.processInputs("Hood", inputs);
        }
    
        motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isMotorConnected) && !Robot.isJITing());

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get());
        }

        if (kMaximumVelocityRadiansPerSecond.hasChanged(hashCode()) || kMaximumAccelerationRadiansPerSecond2.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    kMaximumVelocityRadiansPerSecond.get(), 
                    kMaximumAccelerationRadiansPerSecond2.get()
                )
            );
        }

        wantsToDeploy = !isHomed || (getMeasuredAngleRadians() < kMaximumAngle.get() / 2.0);

        // Home on enable
        if (DriverStation.isEnabled() && !isHomed && !homingCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(homingCommand);
        }

        // Run profile
        final boolean shouldRunProfile = !stopProfile
            && brakeModeEnabled.getAsBoolean()
            && (isHomed || GlobalConstants.getRobot() == GlobalConstants.RobotType.SIMBOT)
            && DriverStation.isEnabled();

        Logger.recordOutput("Hood/RunningProfile", shouldRunProfile);

        if (shouldRunProfile) {
            var goalState = new State(MathUtil.clamp(goal.getAngleRadians(), 0.0, kMaximumAngle.get()), 0.0);
            double previousVelocity = setpoint.velocity;
            setpoint = profile.calculate(GlobalConstants.kLoopPeriodSeconds, setpoint, goalState);

            if (setpoint.position < 0.0 || setpoint.position > kMaximumAngle.get()) {
                setpoint = new State(MathUtil.clamp(setpoint.position, 0.0, kMaximumAngle.get()), 0.0);
            }

            atGoal = EqualsUtility.epsilonEquals(setpoint.position, goalState.position) && EqualsUtility.epsilonEquals(setpoint.velocity, goalState.velocity);

            double accel = (setpoint.velocity - previousVelocity) / GlobalConstants.kLoopPeriodSeconds;
            io.setPosition(
                setpoint.position + homedPosition,
                kS.get() * Math.signum(setpoint.velocity) + kG.get() * Math.cos(setpoint.position) + kA.get() * accel
            );

            Logger.recordOutput("Hood/Profile/SetpointPositionRadians", setpoint.position);
            Logger.recordOutput("Hood/Profile/SetpointVelocityRadiansPerSecond", setpoint.velocity);
            Logger.recordOutput("Hood/Profile/GoalPositionRadians", goalState.position);
            Logger.recordOutput("Hood/Profile/GoalVelocityRadiansPerSecond", goalState.velocity);
        } else {
            setpoint = new State(getMeasuredAngleRadians(), 0.0);

            Logger.recordOutput("Hood/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("Hood/Profile/SetpointVelocityRadiansPerSecond", 0.0);
            Logger.recordOutput("Hood/Profile/GoalPositionRadians", 0.0);
            Logger.recordOutput("Hood/Profile/GoalVelocityRadiansPerSecond", 0.0);
        }

        Logger.recordOutput("Hood/MeasuredVelocityRadiansPerSecond", inputs.velocity);
        LoggedTracer.record("HoodPeriodicMS");
    }

    public void setGoal(HoodGoal goal) {
        if(goal == this.goal) return;
        this.goal = goal;
    }

    public void setManualVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        if (this.brakeModeEnabled.getAsBoolean() == enabled.getAsBoolean()) return;
        this.brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public void setHome() {
        homedPosition = inputs.position;
        isHomed = true;
    }

    private Command homingSequence() {
        return Commands.startRun(() -> {
            stopProfile = true;
            isHomed = false;
            homingDebouncer = new Debouncer(kHomingTimeoutSeconds.get());
            homingDebouncer.calculate(false);
        }, () -> {
            if (!brakeModeEnabled.getAsBoolean()) return;

            io.setVoltage(kHomingVolts.get());
            isHomed = homingDebouncer.calculate(Math.abs(inputs.velocity) <= kHomingVelocityThreshold.get() && Math.abs(inputs.appliedVoltage) >= kHomingVolts.get() * 0.7);
        }).until(() -> isHomed).andThen(this::setHome).finallyDo(() -> {
            stopProfile = false;
        }).withName("Hood Homing Sequence");
    }

    public void overrideHoming() {
        isHomed = false;
    }

    @AutoLogOutput(key = "Hood/MeasuredAngleRadians")
    public double getMeasuredAngleRadians() {
        return inputs.position - homedPosition;
    }
}
