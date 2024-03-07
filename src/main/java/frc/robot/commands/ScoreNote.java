package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kCartridge;
import frc.robot.Constants.kDeployment;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Deployment;

/**
 * @author
 */
public class ScoreNote extends SequentialCommandGroup {

    public ScoreNote(Deployment sys_deployment, Cartridge sys_cartridge) {
        addCommands(
                // Brings the deployment to the amp position
                Commands.runOnce(() -> sys_deployment.setpoint(kDeployment.setpoints.amp_pos), sys_deployment),
                Commands.waitUntil(
                        // waits until it passes the trigger
                        () -> (Math.abs(sys_deployment.getPosition()) >= Math.abs(kDeployment.setpoints.ampTrigger))),
                // Rolls the cartridge
                Commands.runOnce(() -> sys_cartridge.roll(-kCartridge.voltage), sys_cartridge),
                // Waits until its reached the setpoint
                Commands.waitUntil(
                        () -> (Math.abs(sys_deployment.getPosition() - kDeployment.setpoints.amp_pos) <= 2.0)),
                // go back home
                Commands.runOnce(() -> sys_deployment.setpoint(kDeployment.setpoints.home), sys_deployment),
                // stops the rollers
                Commands.runOnce(() -> sys_cartridge.roll(0), sys_cartridge),
                new WaitCommand(2),
                // stop the motor to save battery
                Commands.runOnce(() -> sys_deployment.stopMot(), sys_deployment));
    }
}