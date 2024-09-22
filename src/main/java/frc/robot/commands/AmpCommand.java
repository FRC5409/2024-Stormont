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
public class AmpCommand extends SequentialCommandGroup {

    private final Deployment sys_deployment;

    public AmpCommand() {
        sys_deployment = Deployment.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_deployment);

        Command command = new SequentialCommandGroup(
            Commands.runOnce(() -> sys_deployment.extendTo(kDeployment.MAX_HEIGHT), sys_deployment),
            Commands.waitUntil(() -> sys_deployment.atSetpoint())
        );

        if (Robot.isSimulation())
            command = command.andThen(NoteVisualizer.generateNoteAmpingVisualizationCommand());

        command = command.andThen(Commands.runOnce(() -> sys_deployment.extendTo(kDeployment.MIN_HEIGHT), sys_deployment));

        addCommands(command);
    }

}
