package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.kCartridge;
import frc.robot.Constants.kClimber;
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
                Commands.runOnce(
                        () ->
                                sys_climber.setPosition(
                                        Constants.kClimber.LOW, Constants.kClimber.KFAST_SLOT),
                        sys_climber),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(
                        () ->
                                Math.abs(sys_climber.getPosition())
                                        >= Math.abs(Constants.kClimber.TRAP_TRIGGER_POS)),
                Commands.waitSeconds(0.2),
                Commands.runOnce(
                        () -> sys_deployment.setPosition(kDeployment.kSetpoints.TRAP_POSITION),
                        sys_cartridge),
                Commands.waitUntil(
                        () -> sys_deployment.atSetpoint(kDeployment.kSetpoints.TRAP_POSITION)),
                Commands.waitSeconds(0.3),
                Commands.runOnce(
                        () -> sys_cartridge.setVoltage(-kCartridge.VOLTAGE), sys_cartridge),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(() -> sys_cartridge.checkir()),
                Commands.waitSeconds(0.3),
                Commands.runOnce(() -> sys_cartridge.setVoltage(0), sys_cartridge),
                Commands.runOnce(
                        () -> sys_deployment.setPosition(kDeployment.kSetpoints.HOME),
                        sys_deployment),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> sys_climber.setPosition(kClimber.MIDDLE, kClimber.KSLOW_SLOT), sys_climber));
    }
}
