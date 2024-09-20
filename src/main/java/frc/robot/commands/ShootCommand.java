package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDeployment;
import frc.robot.Robot;
import frc.robot.subsystems.Deployment;
import frc.robot.utils.NoteVisualizer;

/**
 * @author 
 */
public class ShootCommand extends SequentialCommandGroup {

    private final Deployment sys_deployment;

    public ShootCommand() {
        sys_deployment = Deployment.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_deployment);

        Command command = new SequentialCommandGroup(
            Commands.runOnce(() -> sys_deployment.extendTo(kDeployment.MAX_HEIGHT), sys_deployment),
            Commands.waitUntil(() -> sys_deployment.atSetpoint()),
            Commands.runOnce(() -> sys_deployment.extendTo(kDeployment.MIN_HEIGHT), sys_deployment)
        ).alongWith(
            Commands.runOnce(() -> {Robot.hasNote = false;})
        );

        if (Robot.isSimulation())
            command = command.alongWith(NoteVisualizer.generateNoteVisualizationCommand());

        addCommands(command);
    }

}
