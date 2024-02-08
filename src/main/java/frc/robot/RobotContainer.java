// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeEject;
import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

	// Subsystems
	private final Intake sys_intake;

	// Commands
	private final IntakeNote cmd_intakeNote;
	private final IntakeEject cmd_intakeEject;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Subsystems
		sys_intake = Intake.getInstance();

		// Commands
		cmd_intakeNote = new IntakeNote();
		cmd_intakeEject = new IntakeEject();

		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		m_driverController.leftBumper()
			.whileTrue(cmd_intakeNote);
		
		m_driverController.leftTrigger()
			.whileTrue(cmd_intakeEject);
	}

	/**
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return null;
	}
}
