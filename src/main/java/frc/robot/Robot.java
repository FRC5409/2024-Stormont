// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PhotonVision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;
	private PhotonVision sys_photonVision;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		sys_photonVision = PhotonVision.getInstance();

		// Coast mode after 3 seconds
		new Trigger(this::isEnabled).negate().debounce(5)
				.onTrue(Commands
						.runOnce(() -> m_robotContainer.sys_drivetrain.setAllMotorsNeutralMode(NeutralModeValue.Coast))
						.ignoringDisable(true));
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics that you want ran during disabled, autonomous, teleoperated
	 * and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();

		m_robotContainer.sys_drivetrain.periodic();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		m_robotContainer.sys_climber.setVoltage(0);
	}

	@Override
	public void disabledPeriodic() {
		sys_photonVision.updateCalibrationValues(m_robotContainer.sys_drivetrain);
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {

		m_robotContainer.sys_drivetrain.setAllMotorsNeutralMode(NeutralModeValue.Brake);

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}

		try {
			// If Red
			if (m_robotContainer.isRed()) {
				m_robotContainer.sys_drivetrain
						.setPose(
								PathPlannerAuto
										.getPathGroupFromAutoFile(
												m_robotContainer.sc_autoChooser.getSelected().getName())
										.get(0).flipPath().getPreviewStartingHolonomicPose(),
								PathPlannerAuto.getStaringPoseFromAutoFile(
										m_robotContainer.sc_autoChooser.getSelected().getName()).getRotation(),
								m_robotContainer.isRed());
			} else {
				m_robotContainer.sys_drivetrain.setPose(PathPlannerAuto.getStaringPoseFromAutoFile(
						m_robotContainer.sc_autoChooser.getSelected().getName()), m_robotContainer.isRed());
			}
		} catch (Exception e) {
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		CommandScheduler.getInstance().cancelAll();

		// Stop intake after auto
		m_robotContainer.sys_intake.setVoltage(0);

		m_robotContainer.sys_drivetrain.setAllMotorsNeutralMode(NeutralModeValue.Brake);
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
