package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake.Goal;

public class IntakeFactory {
    public static Command deployGroundIntakeNonBlocking(RobotContainer robotContainer) {
        return Commands.run(() -> 
            robotContainer.getIntake().setGoal(Goal.DEPLOY), robotContainer.getIntake()
        ).until(robotContainer.getIntake()::pivotAtGoal);
    }

    public static Command parkGroundIntakeNonBlocking(RobotContainer robotContainer) {
        return Commands.run(() -> 
            robotContainer.getIntake().setGoal(Goal.PARKED), robotContainer.getIntake()
        ).until(robotContainer.getIntake()::pivotAtGoal);
    }

    public static Command runRollersIntakingNonBlocking(RobotContainer robotContainer) {
        return Commands.runOnce(() -> robotContainer.getIntake().setRollerVoltage(12), robotContainer.getIntake());
    }

    public static Command runRollersExhaustingNonBlocking(RobotContainer robotContainer) {
        return Commands.runOnce(() -> robotContainer.getIntake().setRollerVoltage(-12), robotContainer.getIntake());
    }

    public static Command stopRollers(RobotContainer robotContainer) {
        return Commands.runOnce(() -> robotContainer.getIntake().stopRoller(), robotContainer.getIntake());
    }
}
