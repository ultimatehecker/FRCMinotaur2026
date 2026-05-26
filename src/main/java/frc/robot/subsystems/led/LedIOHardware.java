package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.GlobalConstants;

public class LedIOHardware implements LedIO {
    private final CANdle candle;
    private final CANdleConfiguration configuration;

    public LedIOHardware() {
        candle = new CANdle(28, GlobalConstants.kCANivoreBus.getParent());
        configuration = new CANdleConfiguration()
            .withLED(
                new LEDConfigs()
                    .withStripType(StripTypeValue.GRBW)
                    .withBrightnessScalar(1.0)   
            )
            .withCANdleFeatures(
                new CANdleFeaturesConfigs()
                    .withEnable5VRail(Enable5VRailValue.Enabled)
            );

        candle.getConfigurator().apply(configuration);
    }

    @Override
    public void setLEDs(Color color) {
        candle.setControl(new SolidColor(0, 111)
            .withColor(new RGBWColor(color))
        );
    }

    @Override
    public void setLEDs(int red, int green, int blue) {
        candle.setControl(new SolidColor(0, 111)
            .withColor(new RGBWColor(red, green, blue))
        );
    }

    @Override
    public void setControl(ControlRequest controlRequest) {
        candle.setControl(controlRequest);
        candle.setControl(new RainbowAnimation(0, 0));
    }

    @Override
    public void clearAnimation() {
        candle.setControl(new EmptyAnimation(0));
    }
}
