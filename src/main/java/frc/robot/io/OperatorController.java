package frc.robot.io;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.minolib.controller.CommandSimulatedXboxController;
import frc.robot.Robot;
import frc.robot.constants.ControllerConstants;

public class OperatorController implements OperatorControllerIO {
    private static OperatorController instance = null;

    public static OperatorController getInstance() {
        if (instance == null) {
            instance = new OperatorController();
        }

        return instance;
    }

    private final CommandXboxController controller;

    private OperatorController() {
        if (Robot.isSimulation()) {
            controller = new CommandSimulatedXboxController(ControllerConstants.kOperatorControllerPort);
        } else {
            controller = new CommandXboxController(ControllerConstants.kOperatorControllerPort);
        }
    }

    @Override
    public XboxController getSecondaryHID() {
        return controller.getHID();
    }

    @Override
    public Trigger selectCloseShootingPreset() {
        return controller.povDown();
    } 

    @Override
    public Trigger selectMediumShootingPreset() {
        return controller.povLeft();
    }

    @Override
    public Trigger selectFarShootingPreset() {
        return controller.povUp();
    }

    @Override
    public Trigger deployClimber() {
        return controller.rightBumper();
    }

    @Override
    public Trigger stowClimber() {
        return controller.leftBumper();
    }
}
