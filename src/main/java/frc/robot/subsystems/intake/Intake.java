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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
import frc.robot.constants.IntakeConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

public class Intake extends SubsystemBase {
    private static final LoggedTunableNumber pivotkP = new LoggedTunableNumber("Intake/Gains/Pivot/kP");
    private static final LoggedTunableNumber pivotkD = new LoggedTunableNumber("Intake/Gains/Pivot/kD");
    private static final LoggedTunableNumber pivotkS = new LoggedTunableNumber("Intake/Gains/Pivot/kS");
    private static final LoggedTunableNumber pivotkG = new LoggedTunableNumber("Intake/Gains/Pivot/kG");
    private static final LoggedTunableNumber pivotkA = new LoggedTunableNumber("Intake/Gains/Pivot/kA");
    
    private static final LoggedTunableNumber kPivotMaximumAngle = new LoggedTunableNumber("Intake/Pivot/MaximumAngle", IntakeConstants.kIntakeMaximumPosition.in(Radians));
    private static final LoggedTunableNumber kMaximumVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Pivot/MaxVelocityRadiansPerSecond", IntakeConstants.kPivotMaximumRotationalVelocity.in(RadiansPerSecond));
    private static final LoggedTunableNumber kMaximumAccelerationRadiansPerSecond2 = new LoggedTunableNumber("Intake/Pivot/MaxAccelerationRadiansPerSecond2", IntakeConstants.kPivotMaximumRotationalAcceleration.in(RadiansPerSecondPerSecond));
    private static final LoggedTunableNumber kHomingVoltage = new LoggedTunableNumber("Intake/Pivot/HomingVoltage", -3.0);
    private static final LoggedTunableNumber kHomingTimeoutSeconds = new LoggedTunableNumber("Intake/Pivot/HomingTimeSeconds", 0.4);
    private static final LoggedTunableNumber kHomingVelocityThreshold = new LoggedTunableNumber("Intake/Pivot/HomingVelocityThreshold", 0.1);

    @RequiredArgsConstructor
    public enum PivotGoal {
        DEPLOY(new LoggedTunableNumber("Intake/Pivot/DeployedDegrees", IntakeConstants.kIntakeMaximumPosition.in(Degrees))),
        PARKED(new LoggedTunableNumber("Intake/Pivot/ParkedDegrees", IntakeConstants.kIntakeMinimumPosition.in(Degrees))),
        FEED(new LoggedTunableNumber("Intake/Pivot/FeedDegrees", 110.0)),
        CLIMB(new LoggedTunableNumber("Intake/Pivot/ClimbDegrees", 70.0));

        private final DoubleSupplier positionDegrees;

        public double getAngleRadians() {
            return Units.degreesToRadians(positionDegrees.getAsDouble());
        }
    }

    @RequiredArgsConstructor
    public enum RollerGoal {
        INTAKE(new LoggedTunableNumber("Intake/Roller/IntakeVoltage", 12.0)),
        EXHAUST(new LoggedTunableNumber("Intake/Roller/ExhaustVoltage", -6.0)),
        IDLE(new LoggedTunableNumber("Intake/Roller/IdleVoltage", 2.0)),
        STOP(new LoggedTunableNumber("Intake/Roller/StopVoltage", 0.0));

        private final DoubleSupplier rollerVoltage;

        public double getRollerVoltage() {
            return rollerVoltage.getAsDouble();
        }
    }

    public enum PivotControlMode {
        COAST,
        OPEN_LOOP,
        CLOSED_LOOP
    }

    public enum RollerControlMode {
        COAST,
        OPEN_LOOP,
        CLOSED_LOOP
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                pivotkP.initDefault(IntakeConstants.pivotKp);
                pivotkD.initDefault(IntakeConstants.pivotKd);
                pivotkS.initDefault(IntakeConstants.pivotKs);
                pivotkG.initDefault(IntakeConstants.pivotKCos);
                pivotkA.initDefault(IntakeConstants.pivotKa);
            }
            case SIMBOT -> {
                pivotkP.initDefault(IntakeConstants.pivotSimulatedKp);
                pivotkD.initDefault(IntakeConstants.pivotSimulatedKd);
                pivotkS.initDefault(IntakeConstants.pivotSimulatedKs);
                pivotkG.initDefault(IntakeConstants.pivotSimulatedKCos);
                pivotkA.initDefault(IntakeConstants.pivotSimulatedKa);
            }
        }
    }

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final Debouncer pivotMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer rollerMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Alert pivotMotorDisconnectedAlert = new Alert("Intake pivot motor disconnected!", AlertType.kError);
    private final Alert rollerMotorDisconnectedAlert = new Alert("Intake roller motor disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Intake/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> true;

    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    @Getter private PivotGoal pivotGoal = PivotGoal.PARKED;
    @Getter private RollerGoal rollerGoal = RollerGoal.STOP;
    private boolean stopProfile = false;

    @AutoLogOutput(key = "Intake/Pivot/ControlMode")
    private PivotControlMode pivotControlMode = PivotControlMode.CLOSED_LOOP;
    private double pivotOpenLoopVoltage = 0.0;

    @AutoLogOutput(key = "Intake/Roller/ControlMode")
    private RollerControlMode rollerControlMode = RollerControlMode.OPEN_LOOP;
    private double rollerOpenLoopVoltage = 0.0;

    @AutoLogOutput(key = "Intake/Pivot/HomedPositionRadians")
    private double pivotHomedPosition = 0.0;

    @AutoLogOutput(key = "Intake/Pivot/Homed")
    @Getter private boolean pivotHomed = true;

    private Debouncer pivotHomingDebouncer = new Debouncer(kHomingTimeoutSeconds.get());

    @Getter
    @Accessors(fluent = true)
    @AutoLogOutput(key = "Intake/Pivot/Profile/AtGoal")
    private boolean pivotAtGoal = false;

    @Getter
    @Accessors(fluent = true)
    private boolean pivotWantsToDeploy = false;

    public Intake(IntakeIO io) {
        this.io = io;

        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                kMaximumVelocityRadiansPerSecond.get(), 
                kMaximumAccelerationRadiansPerSecond2.get()
            )
        );

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
            Logger.processInputs("Intake", inputs);
        }

        pivotMotorDisconnectedAlert.set(!pivotMotorConnectedDebouncer.calculate(inputs.pivotMotorConnected) && !Robot.isJITing());
        rollerMotorDisconnectedAlert.set(!rollerMotorConnectedDebouncer.calculate(inputs.rollerMotorConnected) && !Robot.isJITing());

        // Update tunable numbers
        if (pivotkP.hasChanged(hashCode()) || pivotkD.hasChanged(hashCode())) {
            io.setPivotPID(pivotkP.get(), 0.0, pivotkD.get());
        }

        if (kMaximumVelocityRadiansPerSecond.hasChanged(hashCode()) || kMaximumAccelerationRadiansPerSecond2.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    kMaximumVelocityRadiansPerSecond.get(), 
                    kMaximumAccelerationRadiansPerSecond2.get()
                )
            );
        }

        pivotWantsToDeploy = !pivotHomed || (getPivotMeasuredAngleRadians() < kPivotMaximumAngle.get() / 2.0);

        // Home on enable
        if (DriverStation.isEnabled() && !pivotHomed && !backupHomingSequence().isScheduled()) {
            CommandScheduler.getInstance().schedule(backupHomingSequence());
        }
        
        switch (pivotControlMode) {
            case COAST -> {
                io.setPivotVoltage(0.0);
                setpoint = new State(getPivotMeasuredAngleRadians(), 0.0);
                pivotAtGoal = false;

                //io.setBrakeMode(true); // TODO: Seperate roller and pivot into their own functions to brake them individually
            }

            case OPEN_LOOP -> {
                io.setPivotVoltage(pivotOpenLoopVoltage);
                setpoint = new State(getPivotMeasuredAngleRadians(), 0.0);
                pivotAtGoal = false;
            }

            case CLOSED_LOOP -> {
                final boolean shouldRunProfile = !stopProfile
                    && brakeModeEnabled.getAsBoolean()
                    && (pivotHomed || GlobalConstants.getRobot() == GlobalConstants.RobotType.SIMBOT)
                    && DriverStation.isEnabled();

                Logger.recordOutput("Intake/Pivot/RunningProfile", shouldRunProfile);

                if (shouldRunProfile) {
                    var goalState = new State(MathUtil.clamp(pivotGoal.getAngleRadians(), 0.0, kPivotMaximumAngle.get()), 0.0);
                    double previousVelocity = setpoint.velocity;
                    setpoint = profile.calculate(GlobalConstants.kLoopPeriodSeconds, setpoint, goalState);

                    if (setpoint.position < 0.0 || setpoint.position > kPivotMaximumAngle.get()) {
                        setpoint = new State(MathUtil.clamp(setpoint.position, 0.0, kPivotMaximumAngle.get()), 0.0);
                    }

                    pivotAtGoal = EqualsUtility.epsilonEquals(setpoint.position, goalState.position) && EqualsUtility.epsilonEquals(setpoint.velocity, goalState.velocity);

                    double accel = (setpoint.velocity - previousVelocity) / GlobalConstants.kLoopPeriodSeconds;
                    io.setPivotPosition(
                        setpoint.position + pivotHomedPosition,
                        pivotkS.get() * Math.signum(setpoint.velocity) + pivotkG.get() * Math.cos(setpoint.position) + pivotkA.get() * accel
                    );

                    Logger.recordOutput("Intake/Pivot/Profile/SetpointPositionRadians", setpoint.position);
                    Logger.recordOutput("Intake/Pivot/Profile/SetpointVelocityRadiansPerSecond", setpoint.velocity);
                    Logger.recordOutput("Intake/Pivot/Profile/GoalPositionRadians", goalState.position);
                    Logger.recordOutput("Intake/Pivot/Profile/GoalVelocityRadiansPerSecond", goalState.velocity);
                } else {
                    setpoint = new State(getPivotMeasuredAngleRadians(), 0.0);

                    Logger.recordOutput("Intake/Pivot/Profile/SetpointPositionMeters", 0.0);
                    Logger.recordOutput("Intake/Pivot/Profile/SetpointVelocityRadiansPerSecond", 0.0);
                    Logger.recordOutput("Intake/Pivot/Profile/GoalPositionRadians", 0.0);
                    Logger.recordOutput("Intake/Pivot/Profile/GoalVelocityRadiansPerSecond", 0.0);
                }
            }
        }

        switch (rollerControlMode) {
            case COAST -> {
                //io.setBrakeMode(true);
            }

            case OPEN_LOOP -> {
                double voltage = switch (rollerGoal) {
                    case INTAKE, EXHAUST, IDLE -> rollerGoal.getRollerVoltage();
                    case STOP -> 0.0;
                };

                io.setRollerVoltage(voltage);
                Logger.recordOutput("Intake/Roller/ControlMode", "OPEN_LOOP");
            }
        }

        Logger.recordOutput("Intake/Pivot/MeasuredVelocityRadiansPerSecond", inputs.pivotVelocity);

        Robot.batteryLogger.reportCurrentUsage(
            "Intake/Pivot",
            inputs.pivotMotorConnected ? inputs.pivotSupplyCurrentAmperes : 0.0
        );

        Robot.batteryLogger.reportCurrentUsage(
            "Intake/Roller",
            inputs.rollerMotorConnected ? inputs.rollerSupplyCurrentAmperes : 0.0
        );

        LoggedTracer.record("IntakePeriodic");
    }

    private void setPivotControlMode(PivotControlMode mode) {
        if (mode == pivotControlMode) return;
        if (mode == PivotControlMode.CLOSED_LOOP) setpoint = new State(getPivotMeasuredAngleRadians(), 0.0);
        
        pivotControlMode = mode;
    }

    private void setRollerControlMode(RollerControlMode mode) {
        rollerControlMode = mode;
    }

    public void setPivotSetpoint(PivotGoal goal) {
        if (pivotControlMode == PivotControlMode.CLOSED_LOOP) setPivotControlMode(PivotControlMode.CLOSED_LOOP);
        if (goal == this.pivotGoal) return;

        this.pivotGoal = goal;
        pivotAtGoal = false;
    }

    public void setPivotOpenLoopVoltage(double voltage) {
        pivotControlMode = PivotControlMode.OPEN_LOOP;
        pivotOpenLoopVoltage = voltage;
    }

    public void setRollerOpenLoopVoltage(RollerGoal goal) {
        setRollerControlMode(RollerControlMode.OPEN_LOOP);
        this.rollerGoal = goal;
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        if (this.brakeModeEnabled.getAsBoolean() == enabled.getAsBoolean()) return;
        this.brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public void setHome() {
        pivotHomedPosition = inputs.pivotPosition;
        pivotHomed = true;
    }

    private Command setPivotGoalCommand(PivotGoal goal) {
        return Commands.runOnce(() -> setPivotSetpoint(goal));
    }

    private Command backupHomingSequence() {
        return Commands.startRun(() -> {
            stopProfile = true;
            pivotHomed = false;
            pivotHomingDebouncer = new Debouncer(kHomingTimeoutSeconds.get());
            pivotHomingDebouncer.calculate(false);
        }, () -> {
            if (!brakeModeEnabled.getAsBoolean()) return;
            io.setPivotVoltage(kHomingVoltage.get());
            pivotHomed = pivotHomingDebouncer.calculate(Math.abs(inputs.pivotVelocity) <= kHomingVelocityThreshold.get() && Math.abs(inputs.pivotAppliedVoltage) >= kHomingVoltage.get() * 0.7);
        }).until(() -> pivotHomed).andThen(this::setHome).finallyDo(() -> {
            stopProfile = false;
        }).withName("Intake Homing Sequence");
    }

    public Command prepareToClimb() {
        return setPivotGoalCommand(PivotGoal.CLIMB);
    }

    public void overrideHoming() {
        pivotHomed = false;
    }

    @AutoLogOutput(key = "Intake/Pivot/MeasuredAngleRads")
    public double getPivotMeasuredAngleRadians() {
        return inputs.pivotPosition - pivotHomedPosition;
    }

    public double getRollerTorqueCurrent() {
        return inputs.rollerTorqueCurrentAmperes;
    }

    public double getRollerVelocity() {
        return inputs.rollerVelocity;
    }
}
