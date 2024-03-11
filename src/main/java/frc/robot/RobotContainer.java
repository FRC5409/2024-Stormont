// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kCartridge;
import frc.robot.Constants.kControllers;
import frc.robot.Constants.kDeployment;
import frc.robot.Constants.kDrive;
import frc.robot.commands.BringNoteToCartridge;
import frc.robot.commands.IntakeToDeploy;
import frc.robot.commands.ScoreNote;
import frc.robot.commands.ScoreTrap;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Deployment;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
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

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        // Joysticks
        private final CommandXboxController m_primaryController;
        private final CommandXboxController m_secondaryController;
        private final CommandXboxController m_testController;

        // Subsystems
        public final Drivetrain sys_drivetrain;
        public final Climber sys_climber;
        public final Intake sys_intake;
        public final Indexer sys_indexer;
        private final Deployment sys_deployment;
        public final Cartridge sys_cartridge;

        // Commands
        private final Command cmd_teleopDrive;

        // SwerveRequest.FieldCentric()
        // .withDeadband(kDrive.kMaxDriveVelocity * 0.1)
        // .withRotationalDeadband(kDrive.kMaxTurnAngularVelocity * 0.1)
        // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Shuffleboard
        public final ShuffleboardTab sb_driveteamTab;

        // Autonomous
        private final SendableChooser<Command> sc_autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // Joysticks
                m_primaryController = new CommandXboxController(kControllers.PRIMARY_CONTROLLER);
                m_secondaryController = new CommandXboxController(kControllers.SECONDARY_CONTROLLER);
                m_testController = new CommandXboxController(kControllers.TEST_CONTROLLER);
                DriverStation.silenceJoystickConnectionWarning(true);

                // Subsystems
                sys_drivetrain = TunerConstants.DriveTrain;
                sys_climber = new Climber();
                sys_intake = Intake.getInstance();
                sys_indexer = Indexer.getInstance();
                sys_deployment = new Deployment();
                sys_cartridge = Cartridge.getInstance();

                // Commands

                cmd_teleopDrive = sys_drivetrain.drive(
                                () -> -m_primaryController.getLeftY() * kDrive.MAX_DRIVE_VELOCIY,
                                () -> -m_primaryController.getLeftX() * kDrive.MAX_DRIVE_VELOCIY,
                                () -> (m_primaryController.getLeftTriggerAxis()
                                                - m_primaryController.getRightTriggerAxis())
                                                * kDrive.MAX_TURN_ANGULAR_VELOCITY);

                sys_drivetrain.setDefaultCommand(cmd_teleopDrive);

                // Shuffleboard
                sb_driveteamTab = Shuffleboard.getTab("Drive team");
                sc_autoChooser = AutoBuilder.buildAutoChooser();
                addShuffleboardItems();

                // Configure the trigger bindings
                configureBindings();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {

                // Primary Controller

                // drivetrain
                m_primaryController.rightBumper()
                                .onTrue(Commands.runOnce(sys_drivetrain::seedFieldRelative, sys_drivetrain));

                // manual deployment extend down
                m_primaryController.povDown()
                                .onTrue(Commands.runOnce(
                                                () -> sys_deployment.manualExtend(Constants.kDeployment.manualVoltage),
                                                sys_deployment))
                                .onFalse(Commands.runOnce(() -> sys_deployment.manualExtend(0), sys_deployment));

                // manual deployment extend up
                m_primaryController.povUp()
                                .onTrue(Commands.runOnce(
                                                () -> sys_deployment.manualExtend(-Constants.kDeployment.manualVoltage),
                                                sys_deployment))
                                .onFalse(Commands.runOnce(() -> sys_deployment.manualExtend(0), sys_deployment));

                // manual cartridge roll backward
                m_primaryController.povRight()
                                .onTrue(Commands.runOnce(() -> sys_cartridge.roll(-Constants.kCartridge.manualVoltage),
                                                sys_cartridge))
                                .onFalse(Commands.runOnce(() -> sys_cartridge.roll(0), sys_cartridge));

                // manual cartridge roll forward
                m_primaryController.povLeft()
                                .onTrue(Commands.runOnce(() -> sys_cartridge.roll(Constants.kCartridge.manualVoltage),
                                                sys_cartridge))
                                .onFalse(Commands.runOnce(() -> sys_cartridge.roll(0), sys_cartridge));

                // manual intake spit
                // m_primaryController.b()
                // .onTrue(Commands.runOnce(() -> sys_intake.setVoltage(-3), sys_intake))
                // .onFalse(Commands.runOnce(() -> sys_intake.setVoltage(0), sys_intake));

                m_primaryController.b()
                                .onTrue(Commands.runOnce(() -> sys_deployment.setpoint(-46), sys_deployment));

                // score command
                m_primaryController.y()
                                .onTrue(new ScoreNote(sys_deployment, sys_cartridge));

                // moves note from intake to deployment
                m_primaryController.x()
                                .onTrue(new IntakeToDeploy(sys_deployment, sys_cartridge, sys_intake, sys_indexer));

                // Secondary Controller
                // *************************************************************************************************************

                // Manual climber movement up
                m_secondaryController.povUp()
                                .onTrue(Commands.runOnce(() -> sys_climber.manualExtend(-Constants.kClimber.VOLTAGE),
                                                sys_climber))
                                .onFalse(Commands.runOnce(() -> sys_climber.manualExtend(0), sys_climber));

                // Manual climber movement down
                m_secondaryController.povDown()
                                .onTrue(Commands.runOnce(() -> sys_climber.manualExtend(Constants.kClimber.VOLTAGE),
                                                sys_climber))
                                .onFalse(Commands.runOnce(() -> sys_climber.manualExtend(0), sys_climber));

                // climber setpoint high
                m_secondaryController.y()
                                .onTrue(Commands.runOnce(() -> sys_climber.setpoint(Constants.kClimber.HIGH),
                                                sys_climber));

                // climber setpoint middle
                m_secondaryController.x()
                                .onTrue(Commands.runOnce(() -> sys_climber.setpoint(Constants.kClimber.MIDDLE),
                                                sys_climber));

                m_secondaryController.a()
                                .onTrue(Commands.runOnce(() -> sys_climber.setpoint(Constants.kClimber.LOW),
                                                sys_climber));

                m_secondaryController.back()
                                .onTrue(new ScoreTrap(sys_deployment, sys_cartridge));

                // deployment extend for scoring, spinning index and cartidgan until ir sensor
                // detection
                // zero climber, extend to high setpoint climber slow and fast, manual climber,
                // extend low setpoint

                // tester controller

        }

        private void addShuffleboardItems() {

                // Re-zero
                sb_driveteamTab.add("Seed field relative",
                                Commands.runOnce(sys_drivetrain::seedFieldRelative, sys_drivetrain))
                                .withPosition(0, 0);

                // Autonomous
                sb_driveteamTab.add("Choose auto", sc_autoChooser)
                                .withPosition(0, 1)
                                .withSize(3, 1);

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return sc_autoChooser.getSelected();
        }
}
