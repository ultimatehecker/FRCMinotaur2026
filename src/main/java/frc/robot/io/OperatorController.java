package frc.robot.io;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
            controller = new CommandSimulatedXboxController(ControllerConstants.kDriverControllerPort);
        } else {
            controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);
        }
    }

    @Override
    public XboxController getSecondaryHID() {
        return controller.getHID();
    }
}
