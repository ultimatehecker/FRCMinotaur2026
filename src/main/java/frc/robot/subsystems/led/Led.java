package frc.robot.subsystems.led;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Led extends SubsystemBase {
    private final LedIO io;
    private final RobotState robotState;

    public Led(LedIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;
        
        setDefaultCommand(commandIdle());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Led/CurrentCommand", (getCurrentCommand() == null) ? "None" : getCurrentCommand().getName());
    }

    public Command commandStaticColor(Color color) {
        return run(() -> io.setLEDs(color))
            .ignoringDisable(true)
            .withName("Led_StaticColor");
    }

    public Command commandStaticColor(Supplier<Color> colorSupplier) {
        return run(() -> io.setLEDs(colorSupplier.get()))
            .ignoringDisable(true)
            .withName("Led_DynamicStaticColor");
    }

    public Command commandAnimation(ControlRequest animationRequest) {
        return run(() -> setControl(animationRequest))
            .ignoringDisable(true)
            .withName("Led_Animation");
    }

    public Command commandBlink(Color color, Time duration) {
        return Commands.sequence(
            runOnce(() -> io.setLEDs(color)),
            Commands.waitTime(duration),
            runOnce(() -> io.clearAnimation()),
            Commands.waitTime(duration)
        )
        .repeatedly()
        .ignoringDisable(true)
        .withName("Led_Blink");
    }

    public Command commandClear() {
        return runOnce(() -> io.clearAnimation())
            .ignoringDisable(true)
            .withName("Led_Clear");
    }

    public Command commandIdle() {
        return run(() -> setControl(
            new LarsonAnimation(0, 111)
                .withColor(robotState.isRedAlliance() ? new RGBWColor(Color.kRed) : new RGBWColor(Color.kBlue))
                .withBounceMode(LarsonBounceValue.Back)
        ))
            .ignoringDisable(true)
            .withName("Led_Animation");
    }

    public void setControl(ControlRequest request) {
        io.setControl(request);
    }
}