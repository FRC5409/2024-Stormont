package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDeployment;
import frc.robot.subsystems.Deployment;

/**
 * @author 
 */
public class ShootCommand extends SequentialCommandGroup {

    private final Deployment sys_deployment;

    public ShootCommand() {
        sys_deployment = Deployment.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sys_deployment);

        addCommands(
            Commands.runOnce(() -> sys_deployment.extendTo(kDeployment.MAX_HEIGHT), sys_deployment),
            Commands.waitUntil(() -> sys_deployment.atSetpoint()),
            Commands.runOnce(() -> sys_deployment.extendTo(kDeployment.MIN_HEIGHT), sys_deployment)
        );
    }

}
