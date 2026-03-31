package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake.PivotGoal;
import frc.robot.subsystems.intake.Intake.RollerGoal;

public class IntakeFactory {
    public static Command prepareIntakeBlocking(RobotContainer robotContainer) {
        return Commands.parallel(
            Commands.runOnce(() -> robotContainer.getIntake().setPivotSetpoint(PivotGoal.DEPLOY)).until(robotContainer.getIntake()::pivotAtGoal),
            Commands.startEnd(
                () -> robotContainer.getIntake().setRollerOpenLoopVoltage(RollerGoal.INTAKE),
                () -> robotContainer.getIntake().setRollerOpenLoopVoltage(RollerGoal.STOP)
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
    }

    public static Command parkIntakeBlocking(RobotContainer robotContainer) {
        return Commands.parallel(
            Commands.runOnce(() -> robotContainer.getIntake().setPivotSetpoint(PivotGoal.PARKED)),
            Commands.runOnce(() -> robotContainer.getIntake().setRollerOpenLoopVoltage(RollerGoal.STOP))
        ).until(robotContainer.getIntake()::pivotAtGoal).withName("Retract Intake");
    }
}
