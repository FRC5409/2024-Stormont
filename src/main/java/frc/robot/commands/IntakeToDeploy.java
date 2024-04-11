// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kCartridge;
import frc.robot.Constants.kDeployment;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kIntake;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Deployment;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeToDeploy extends SequentialCommandGroup {
	/** Creates a new IntakeToDeploy. */
	public IntakeToDeploy(Deployment sys_deployment, Cartridge sys_cartridge, Intake sys_intake, Indexer sys_indexer) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				Commands.runOnce(() -> sys_intake.setVoltage(kIntake.VOLTAGE), sys_intake).alongWith(
						Commands.runOnce(() -> sys_indexer.setVoltage(kIndexer.VOLTAGE), sys_indexer),
						Commands.runOnce(() -> sys_cartridge.setVoltage(kCartridge.VOLTAGE), sys_cartridge)),
				Commands.waitUntil(() -> sys_indexer.checkIR()),
				Commands.runOnce(() -> sys_intake.setVoltage(0), sys_intake).alongWith(
						Commands.runOnce(() -> sys_indexer.setVoltage(0), sys_indexer),
						Commands.runOnce(() -> sys_cartridge.setVoltage(0), sys_cartridge)),
				Commands.runOnce(
						() -> sys_deployment.setPosition(kDeployment.kSetpoints.LOW, kDeployment.kPID.kFastSlot.slot),
						sys_deployment),
				Commands.waitUntil(() -> sys_deployment.atSetpoint(kDeployment.kSetpoints.LOW)),
				Commands.waitSeconds(1), new BringNoteToCartridge(sys_cartridge, sys_indexer));
	}
}
