// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kCartridge;
import frc.robot.Constants.kClimber;
import frc.robot.Constants.kControllers;
import frc.robot.Constants.kDeployment;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kDrive.kAutoAlign;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kRobot;
import frc.robot.Constants.kWaypoints;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.BringNoteToCartridge;
import frc.robot.commands.ScoreNote;
import frc.robot.commands.ScoreTrap;
import frc.robot.generated.TunerConstantsBeta;
import frc.robot.generated.TunerConstantsComp;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Deployment;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonVision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	// Joysticks
	private final CommandXboxController m_primaryController;
	private final CommandXboxController m_secondaryController;
	private final CommandXboxController m_calibrationController;

	// Subsystems
	public final Drivetrain sys_drivetrain;
	public final Climber sys_climber;
	public final Intake sys_intake;
	public final Indexer sys_indexer;
	private final Deployment sys_deployment;
	public final Cartridge sys_cartridge;
	public final PhotonVision sys_photonvision;

	// Commands
	private final Command cmd_teleopDrive;
	private final Command cmd_intakeToSensor;

	// Shuffleboard
	public final ShuffleboardTab sb_driveteamTab;

	// Autonomous
	public final SendableChooser<Command> sc_autoChooser;
	public final SendableChooser<Boolean> sc_alliance;
	private final GenericEntry sb_autoDelay;
	private final GenericEntry sb_trapOffset;
	private final GenericEntry sb_ampOffset;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// Joysticks
		m_primaryController = new CommandXboxController(kControllers.PRIMARY_CONTROLLER);
		m_secondaryController = new CommandXboxController(kControllers.SECONDARY_CONTROLLER);
		m_calibrationController = new CommandXboxController(kControllers.CALIBRATION_CONTROLLER);
		DriverStation.silenceJoystickConnectionWarning(true);

		// Subsystems
		sys_climber = new Climber();
		sys_intake = Intake.getInstance();
		sys_indexer = Indexer.getInstance();
		sys_deployment = new Deployment();
		sys_cartridge = Cartridge.getInstance();
		sys_photonvision = PhotonVision.getInstance();

		// Subsystems
		sys_drivetrain = kRobot.IS_BETA_ROBOT ? TunerConstantsBeta.DriveTrain : TunerConstantsComp.DriveTrain;

		// Commands
		cmd_teleopDrive = sys_drivetrain.drive(() -> -m_primaryController.getLeftY() * kDrive.MAX_DRIVE_VELOCIY,
				() -> -m_primaryController.getLeftX() * kDrive.MAX_DRIVE_VELOCIY,
				() -> (m_primaryController.getLeftTriggerAxis() - m_primaryController.getRightTriggerAxis())
						* kDrive.MAX_TURN_ANGULAR_VELOCITY);

		cmd_intakeToSensor = Commands.runOnce(() -> {
			sys_intake.setVoltage(kIntake.VOLTAGE);
			sys_indexer.setVoltage(kIndexer.VOLTAGE);
		}, sys_intake, sys_indexer).andThen(Commands.waitUntil(() -> sys_intake.checkIR()))
				.andThen(Commands.runOnce(() -> {
					sys_intake.setVoltage(0);
					sys_indexer.setVoltage(0);
				}, sys_intake, sys_indexer));

		sys_drivetrain.setDefaultCommand(cmd_teleopDrive);

		registerPathplannerCommands();

		// Shuffleboard
		sb_driveteamTab = Shuffleboard.getTab("Drive team");
		sc_autoChooser = AutoBuilder.buildAutoChooser();

		sc_alliance = new SendableChooser<>();
		sc_alliance.addOption("Blue", false);
		sc_alliance.addOption("Red", true);
		sc_alliance.setDefaultOption("Blue", false);

		sb_autoDelay = sb_driveteamTab.add("Auto delay", 0.0).withPosition(4, 0).withSize(1, 1).getEntry();

		// Autonomous
		sb_driveteamTab.add("Choose auto", sc_autoChooser).withPosition(0, 0).withSize(3, 1);
		sb_driveteamTab.add("Alliance", sc_alliance).withPosition(0, 1).withSize(3, 1);

		// Automations
		sb_trapOffset = sb_driveteamTab.add("Trap Offset", 0).withPosition(5, 0).withSize(1, 1).getEntry();
		sb_ampOffset = sb_driveteamTab.add("Amp X Offset", 0).withPosition(5, 1).withSize(1, 1).getEntry();

		// Configure the trigger bindings
		configureBindings();

		new Trigger(() -> sys_indexer.checkIR()).and(DriverStation::isTeleop)
				.onTrue(new BringNoteToCartridge(sys_cartridge, sys_indexer).onlyIf(DriverStation::isTeleop))
				.onTrue(Commands.runOnce(() -> m_primaryController.getHID().setRumble(RumbleType.kBothRumble, 0.3))
						.andThen(new WaitCommand(0.75)).andThen(Commands
								.runOnce(() -> m_primaryController.getHID().setRumble(RumbleType.kBothRumble, 0.0))));
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
	 * for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
	 * Flight joysticks}.
	 */
	private void configureBindings() {

		// Primary Controller

		// drivetrain
		m_primaryController.rightBumper()
				.onTrue(Commands.runOnce(sys_drivetrain::seedFieldRelative, sys_drivetrain).ignoringDisable(true));

		// score command
		m_primaryController.a().onTrue(new ScoreNote(sys_deployment, sys_cartridge));

		// Intake note command
		m_primaryController.x().whileTrue(Commands.race(Commands.startEnd(() -> {
			sys_intake.setVoltage(kIntake.VOLTAGE);
			sys_indexer.setVoltage(kIndexer.VOLTAGE);
		}, () -> {
			sys_intake.setVoltage(0);
			sys_indexer.setVoltage(0);
		}, sys_intake, sys_indexer), Commands.waitUntil(() -> sys_indexer.checkIR())));

		// Eject note command
		m_primaryController.b().onTrue(Commands.runOnce(() -> {
			sys_cartridge.setVoltage(-kCartridge.VOLTAGE);
			sys_intake.setVoltage(-12.0);
		}, sys_cartridge, sys_intake)).onFalse(Commands.runOnce(() -> {
			sys_cartridge.setVoltage(0);
			sys_intake.setVoltage(0);
		}, sys_cartridge, sys_intake));

		m_primaryController.y().onTrue(cmd_intakeToSensor).onFalse(Commands.runOnce(() -> {
			sys_intake.setVoltage(0);
			sys_indexer.setVoltage(0);
		}, sys_intake, sys_indexer));

		m_primaryController.start().onTrue(new BringNoteToCartridge(sys_cartridge, sys_indexer));

		m_primaryController.back().onTrue(Commands.runOnce(() -> sys_cartridge.setVoltage(kCartridge.VOLTAGE)))
				.onFalse(Commands.runOnce(() -> sys_cartridge.setVoltage(0.0)));

		m_primaryController.leftBumper()
				.onTrue(new BringNoteToCartridge(sys_cartridge, sys_indexer)
						.andThen(Commands.runOnce(() -> sys_deployment.setPosition(kDeployment.kSetpoints.AMP_POSITION,
								kDeployment.kPID.kFastSlot.slot), sys_deployment)))
				// .whileTrue(new AlignToPose(sys_drivetrain.getAmpWaypoint(), sys_drivetrain));
				.whileTrue(new AlignToPose(() -> sys_drivetrain.getAmpWaypoint(this::isRed, sb_ampOffset.getDouble(0)),
						sys_drivetrain, true, kAutoAlign.REACHED_POSITION_TIMEOUT_SLOW,
						kAutoAlign.REACHED_POSITION_TOLERANCE_ClOSE, this::isRed));

		// Secondary Controller
		// *************************************************************************************************************

		// Climber setpoint high fast
		m_secondaryController.povUp().onTrue(Commands.runOnce(
				() -> sys_climber.setPosition(Constants.kClimber.HIGH, Constants.kClimber.KFAST_SLOT), sys_climber));

		// Climber setpoint high slow
		m_secondaryController.povLeft()
				// .onTrue(
				// Commands.runOnce(
				// () ->
				// sys_climber.setPosition(
				// Constants.kClimber.HIGH,
				// Constants.kClimber.KSLOW_SLOT),
				// sys_climber));
				.onTrue(Commands.runOnce(() -> sys_climber.setPosition(kClimber.HIGH, kClimber.KFAST_SLOT), sys_climber)
						.andThen(Commands.waitUntil(
								() -> Math.abs(sys_climber.getPosition()) >= Math.abs(kClimber.SLOW_TRIGGER)))
						.andThen(Commands.runOnce(() -> sys_climber.setPosition(kClimber.HIGH, kClimber.KSLOW_SLOT),
								sys_climber)));

		// Climber setpoint low fast
		m_secondaryController.povDown().onTrue(Commands.runOnce(
				() -> sys_climber.setPosition(Constants.kClimber.LOW, Constants.kClimber.KFAST_SLOT), sys_climber));

		// Climber manual down
		m_secondaryController.leftBumper()
				.onTrue(Commands.runOnce(() -> sys_climber.setVoltage(Constants.kClimber.VOLTAGE), sys_climber))
				.onFalse(Commands.runOnce(() -> sys_climber.setVoltage(0), sys_climber));

		// Climber manual up
		m_secondaryController.rightBumper()
				.onTrue(Commands.runOnce(() -> sys_climber.setVoltage(-Constants.kClimber.VOLTAGE), sys_climber))
				.onFalse(Commands.runOnce(() -> sys_climber.setVoltage(0), sys_climber));

		// turns off climber pid
		m_secondaryController.start().onTrue(Commands.runOnce(() -> sys_climber.setVoltage(0), sys_climber));

		// Bring note from indexer to cartridge, when stuck
		m_secondaryController.back()
				.onTrue(Commands.runOnce(() -> sys_drivetrain.setDriveMotorInversions(), sys_drivetrain));

		// Climb, extend and score, endgame sequence
		m_secondaryController.y().onTrue(new ScoreTrap(sys_deployment, sys_cartridge, sys_climber));

		m_secondaryController.x().onTrue(Commands.runOnce(() -> {
			sys_photonvision.setCameraEnableStatus(false, "Front");
			sys_photonvision.setCameraEnableStatus(false, "Back");
			sys_photonvision.setCameraEnableStatus(true, "Top");
		}, sys_photonvision)).whileTrue(new AlignToPose(() -> {
			return sys_photonvision.getNearestTagPoseWithOffset(sys_drivetrain,
					kWaypoints.TRAP_DISTANT_OFFSET + sb_trapOffset.getDouble(0),
					sys_drivetrain.getTrapRotation(this::isRed, 1));
		}, sys_drivetrain, true, kAutoAlign.REACHED_POSITION_TIMEOUT_FAST, kAutoAlign.REACHED_POSITION_TOLERANCE,
				this::isRed).andThen(new AlignToPose(() -> {
					return sys_photonvision.getNearestTagPoseWithOffset(sys_drivetrain,
							kWaypoints.TRAP_OFFSET + sb_trapOffset.getDouble(0),
							sys_drivetrain.getTrapRotation(this::isRed, 1));
				}, sys_drivetrain, true, kAutoAlign.REACHED_POSITION_TIMEOUT_SLOW,
						kAutoAlign.REACHED_POSITION_TOLERANCE_ClOSE, this::isRed)))
				.onFalse(Commands.runOnce(() -> {
					sys_photonvision.setCameraEnableStatus(true, "Front");
					sys_photonvision.setCameraEnableStatus(true, "Back");
					sys_photonvision.setCameraEnableStatus(false, "Top");
				}, sys_photonvision));

		m_secondaryController.b().onTrue(Commands.runOnce(() -> {
			sys_photonvision.setCameraEnableStatus(false, "Front");
			sys_photonvision.setCameraEnableStatus(false, "Back");
			sys_photonvision.setCameraEnableStatus(true, "Top");
		}, sys_photonvision)).whileTrue(new AlignToPose(() -> {
			return sys_photonvision.getNearestTagPoseWithOffset(sys_drivetrain,
					kWaypoints.TRAP_DISTANT_OFFSET + sb_trapOffset.getDouble(0),
					sys_drivetrain.getTrapRotation(this::isRed, 2));
		}, sys_drivetrain, true, kAutoAlign.REACHED_POSITION_TIMEOUT_FAST, kAutoAlign.REACHED_POSITION_TOLERANCE,
				this::isRed).andThen(new AlignToPose(() -> {
					return sys_photonvision.getNearestTagPoseWithOffset(sys_drivetrain,
							kWaypoints.TRAP_OFFSET + sb_trapOffset.getDouble(0),
							sys_drivetrain.getTrapRotation(this::isRed, 2));
				}, sys_drivetrain, true, kAutoAlign.REACHED_POSITION_TIMEOUT_SLOW,
						kAutoAlign.REACHED_POSITION_TOLERANCE_ClOSE, this::isRed)))
				.onFalse(Commands.runOnce(() -> {
					sys_photonvision.setCameraEnableStatus(true, "Front");
					sys_photonvision.setCameraEnableStatus(true, "Back");
					sys_photonvision.setCameraEnableStatus(false, "Top");
				}, sys_photonvision));

		m_secondaryController.a().onTrue(Commands.runOnce(() -> {
			sys_photonvision.setCameraEnableStatus(false, "Front");
			sys_photonvision.setCameraEnableStatus(false, "Back");
			sys_photonvision.setCameraEnableStatus(true, "Top");
		}, sys_photonvision)).whileTrue(new AlignToPose(() -> {
			return sys_photonvision.getNearestTagPoseWithOffset(sys_drivetrain,
					kWaypoints.TRAP_DISTANT_OFFSET + sb_trapOffset.getDouble(0),
					sys_drivetrain.getTrapRotation(this::isRed, 3));
		}, sys_drivetrain, true, kAutoAlign.REACHED_POSITION_TIMEOUT_FAST, kAutoAlign.REACHED_POSITION_TOLERANCE,
				this::isRed).andThen(new AlignToPose(() -> {
					return sys_photonvision.getNearestTagPoseWithOffset(sys_drivetrain,
							kWaypoints.TRAP_OFFSET + sb_trapOffset.getDouble(0),
							sys_drivetrain.getTrapRotation(this::isRed, 3));
				}, sys_drivetrain, true, kAutoAlign.REACHED_POSITION_TIMEOUT_SLOW,
						kAutoAlign.REACHED_POSITION_TOLERANCE_ClOSE, this::isRed)))
				.onFalse(Commands.runOnce(() -> {
					sys_photonvision.setCameraEnableStatus(true, "Front");
					sys_photonvision.setCameraEnableStatus(true, "Back");
					sys_photonvision.setCameraEnableStatus(false, "Top");
				}, sys_photonvision));

		// CALIBRATION CONTROLLER
		m_calibrationController.x().onTrue(Commands.runOnce(() -> {
			sys_photonvision.setCameraEnableStatus(false, "Front");
			sys_photonvision.setCameraEnableStatus(false, "Back");
			sys_photonvision.setCameraEnableStatus(true, "Top");
		}, sys_photonvision).ignoringDisable(true));

		m_calibrationController.b().onTrue(Commands.runOnce(() -> {
			sys_photonvision.setCameraEnableStatus(true, "Front");
			sys_photonvision.setCameraEnableStatus(true, "Back");
			sys_photonvision.setCameraEnableStatus(false, "Top");
		}, sys_photonvision).ignoringDisable(true));

	}

	public void registerPathplannerCommands() {

		NamedCommands.registerCommand("IntakeFromFloor", Commands.startEnd(() -> {
			sys_intake.setVoltage(kIntake.VOLTAGE);
			sys_indexer.setVoltage(kIndexer.VOLTAGE);
		}, () -> {
			sys_intake.setVoltage(0.0);
			sys_indexer.setVoltage(0.0);
		}, sys_intake, sys_indexer).until(sys_indexer::checkIR));

		NamedCommands.registerCommand("BringNoteToCartridge", new BringNoteToCartridge(sys_cartridge, sys_indexer));
		NamedCommands.registerCommand("ScoreNote", new ScoreNote(sys_deployment, sys_cartridge).withTimeout(1.0));

		NamedCommands.registerCommand("EjectNote", Commands.startEnd(() -> {
			sys_cartridge.setVoltage(-kCartridge.VOLTAGE);
			sys_intake.setVoltage(-12.0);
		}, () -> {
			sys_cartridge.setVoltage(0.0);
			sys_intake.setVoltage(0.0);
		}, sys_cartridge, sys_intake).withTimeout(1.0));

		NamedCommands.registerCommand("IntakeToSensor", cmd_intakeToSensor);

		NamedCommands.registerCommand("EjectPreload", Commands.waitSeconds(0.6).andThen(
				Commands.runOnce(() -> sys_cartridge.setVoltage(5), sys_cartridge),
				Commands.runOnce(() -> sys_deployment.setPosition(-25, kDeployment.kPID.kSlowSlot.slot),
						sys_deployment),
				Commands.waitSeconds(0.4), Commands.runOnce(() -> sys_cartridge.setVoltage(0), sys_cartridge),
				Commands.runOnce(
						() -> sys_deployment.setPosition(kDeployment.kSetpoints.HOME, kDeployment.kPID.kFastSlot.slot),
						sys_deployment)));

		NamedCommands.registerCommand("EndIfNotInIntake", Commands.waitUntil(() -> !sys_intake.checkIR())
				.andThen(Commands.waitSeconds(0.8)).andThen(Commands.waitUntil(() -> !sys_intake.checkIR())));

		NamedCommands.registerCommand("BringCartridgeUp",
				Commands.runOnce(
						() -> sys_deployment.setPosition(kDeployment.kSetpoints.HOME, kDeployment.kPID.kFastSlot.slot),
						sys_deployment));
	}

	/**
	 * @return True if alliance color that is selected is true
	 */
	public boolean isRed() {
		return sc_alliance.getSelected();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		double delay = sb_autoDelay.getDouble(0);

		if (delay == 0) {
			return sc_autoChooser.getSelected();
		} else {
			return Commands.waitSeconds(delay).andThen(sc_autoChooser.getSelected());
		}
	}
}
