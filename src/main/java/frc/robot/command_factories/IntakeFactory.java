package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake.IntakeGoal;

public class IntakeFactory {
    public static Command intakeCommand(RobotContainer robotContainer) {
        return Commands.startEnd(() -> robotContainer.getIntake().setGoal(IntakeGoal.INTAKE), () -> robotContainer.getIntake().setGoal(IntakeGoal.DEPLOY_HALF));
    }

    public static Command stowCommand(RobotContainer robotContainer) {
        return Commands.runOnce(() -> robotContainer.getIntake().setGoal(IntakeGoal.STOW));
    }

    public static Command deployCommand(RobotContainer robotContainer) {
        return Commands.runOnce(() -> robotContainer.getIntake().setGoal(IntakeGoal.DEPLOY));
    }

    public static Command deployHalfCommand(RobotContainer robotContainer) {
        return Commands.runOnce(() -> robotContainer.getIntake().setGoal(IntakeGoal.DEPLOY_HALF));
    }

    public static Command ejectCommand(RobotContainer robotContainer) {
        return Commands.startEnd(() -> robotContainer.getIntake().setGoal(IntakeGoal.EXHAUST), () -> robotContainer.getIntake().setGoal(IntakeGoal.IDLE));
    }

    public static Command idleCommand(RobotContainer robotContainer) {
        return Commands.runOnce(() -> robotContainer.getIntake().setGoal(IntakeGoal.IDLE));
    }
}
