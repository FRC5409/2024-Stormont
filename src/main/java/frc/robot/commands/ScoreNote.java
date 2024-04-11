package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
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
				Commands.runOnce(
						() -> sys_deployment
								.setPosition(kDeployment.kSetpoints.AMP_POSITION, kDeployment.kPID.kFastSlot.slot),
						sys_deployment),
				Commands.waitUntil(
						// waits until it passes the trigger
						() -> (Math.abs(sys_deployment.getPosition()) >= Math.abs(kDeployment.kSetpoints.AMP_TRIGGER))),
				// Rolls the cartridge
				Commands.runOnce(() -> sys_cartridge.setVoltage(-kCartridge.VOLTAGE), sys_cartridge),
				// Waits until its reached the setpoint
				Commands.waitUntil(() -> (sys_deployment.atSetpoint(kDeployment.kSetpoints.AMP_POSITION))),
				Commands.waitSeconds(0.45),

				// go back home
				Commands.runOnce(
						() -> sys_deployment.setPosition(kDeployment.kSetpoints.HOME, kDeployment.kPID.kFastSlot.slot),
						sys_deployment),
				// stops the rollers
				Commands.runOnce(() -> sys_cartridge.setVoltage(0), sys_cartridge),
				new WaitCommand(1).onlyIf(DriverStation::isTeleop),
				// stop the motor to save battery
				Commands.runOnce(() -> sys_deployment.stopMot(), sys_deployment).onlyIf(DriverStation::isTeleop));
	}
}
