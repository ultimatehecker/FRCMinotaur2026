package frc.robot.command_factories;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.GlobalConstants;

public class LedFactory {
    public static Command groundIntakeLEDs(RobotContainer container) {
        return container.getLeds().commandBlink(Color.kRed, Milliseconds.of(100));
    }

    public static Command batteryLEDs(RobotContainer container, DoubleSupplier voltageSupplier) {
        return container.getLeds().commandStaticColor(() -> {
            if (voltageSupplier.getAsDouble() < GlobalConstants.kLowBatteryVoltage) {
                return Color.kLightYellow;
            } else {
                return Color.kGreen;
            }
        });
    }
}
