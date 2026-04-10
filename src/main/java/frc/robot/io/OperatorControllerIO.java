package frc.robot.io;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControllerIO {
    public XboxController getSecondaryHID();

    public Trigger selectCloseShootingPreset();

    public Trigger selectMediumShootingPreset();

    public Trigger selectFarShootingPreset();

    public Trigger deployClimber();

    public Trigger stowClimber();
}
