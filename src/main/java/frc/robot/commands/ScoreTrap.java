package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.kCartridge;
import frc.robot.Constants.kDeployment;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Deployment;

/**
 * @author
 */
public class ScoreTrap extends SequentialCommandGroup {

    public ScoreTrap(Deployment sys_deployment, Cartridge sys_cartridge, Climber sys_climber) {
        addCommands(
                Commands.runOnce(() -> sys_climber.setpoint(Constants.kClimber.LOW, Constants.kClimber.KFAST_SLOT),
                        sys_climber),
                Commands.waitUntil(
                        () -> Math.abs(sys_climber.getPosition()) >= Math.abs(Constants.kClimber.trapTriggerPos)),
                Commands.runOnce(() -> sys_deployment.setpoint(kDeployment.setpoints.trap_pos), sys_cartridge),
                Commands.waitUntil(() -> sys_deployment.atSetpoint(kDeployment.setpoints.trap_pos)),
                Commands.waitSeconds(0.3),
                Commands.runOnce(() -> sys_cartridge.roll(-kCartridge.voltage), sys_cartridge),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(() -> sys_cartridge.checkir()),
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> sys_cartridge.roll(0), sys_cartridge),
                Commands.runOnce(() -> sys_deployment.setpoint(kDeployment.setpoints.home), sys_deployment));
    }
}