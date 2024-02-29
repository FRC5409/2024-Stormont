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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.kControllers;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kWaypoints;
import frc.robot.commands.AlignToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
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

        // Subsystems
        public final Drivetrain sys_drivetrain;
        private final Climber sys_climber;
        public final Intake sys_intake;
        public final Indexer sys_indexer;

        // Commands
        private final Command cmd_teleopDrive;

        // Shuffleboard
        public final ShuffleboardTab sb_driveteamTab;

        // Autonomous
        private final SendableChooser<Command> sc_autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // Joysticks
                m_primaryController = new CommandXboxController(kControllers.kPrimaryController);
                m_secondaryController = new CommandXboxController(kControllers.kSecondaryController);
                DriverStation.silenceJoystickConnectionWarning(true);

                // Subsystems
                sys_drivetrain = TunerConstants.DriveTrain;
                sys_climber = new Climber();
                sys_intake = Intake.getInstance();
                sys_indexer = Indexer.getInstance();

                // Commands
                cmd_teleopDrive = sys_drivetrain.drive(
                                () -> -m_primaryController.getLeftY() * kDrive.kMaxDriveVelocity,
                                () -> -m_primaryController.getLeftX() * kDrive.kMaxDriveVelocity,
                                () -> (m_primaryController.getLeftTriggerAxis()
                                                - m_primaryController.getRightTriggerAxis())
                                                * kDrive.kMaxTurnAngularVelocity);

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

                m_primaryController.rightBumper()
                                .onTrue(Commands.runOnce(sys_drivetrain::seedFieldRelative, sys_drivetrain));

                // Manual climber movement up
                m_secondaryController.povUp()
                                .onTrue(Commands.runOnce(() -> sys_climber.manualExtend(-Constants.kClimber.voltage),
                                                sys_climber))
                                .onFalse(Commands.runOnce(() -> sys_climber.manualExtend(0), sys_climber));

                // Manual climber movement down
                m_secondaryController.povDown()
                                .onTrue(Commands.runOnce(() -> sys_climber.manualExtend(Constants.kClimber.voltage),
                                                sys_climber))
                                .onFalse(Commands.runOnce(() -> sys_climber.manualExtend(0), sys_climber));

                m_primaryController.rightBumper()
                                .onTrue(Commands.runOnce(sys_drivetrain::seedFieldRelative, sys_drivetrain));
                m_primaryController.a()
                                .whileTrue(Commands.runOnce(
                                                () -> sys_drivetrain.navigateTo(kWaypoints.kAmpZoneTest,
                                                                m_primaryController),
                                                sys_drivetrain));
                m_primaryController.y() // changed to Y from B to conform to latest button bindings
                                .whileTrue(new AlignToPose(kWaypoints.kAmpZoneTest, sys_drivetrain));

                // Intake note command
                m_primaryController.x()
                                .onTrue(Commands.runOnce(
                                                () -> {
                                                        sys_intake.setRPM(IntakeConstants.RPM);
                                                        sys_indexer.setVoltage(IndexerConstants.VOLTAGE);
                                                }, sys_intake, sys_indexer))
                                .onFalse(Commands.runOnce(
                                                () -> {
                                                        sys_intake.setVoltage(0);
                                                        sys_indexer.setVoltage(0);
                                                }, sys_intake, sys_indexer));

                // Eject note command
                m_primaryController.b()
                                .onTrue(Commands.runOnce(
                                                () -> {
                                                        sys_intake.setRPM(-IntakeConstants.RPM);
                                                        sys_indexer.setVoltage(-IndexerConstants.VOLTAGE);
                                                }, sys_intake, sys_indexer))
                                .onFalse(Commands.runOnce(
                                                () -> {
                                                        sys_intake.setVoltage(0);
                                                        sys_indexer.setVoltage(0);
                                                }, sys_intake, sys_indexer));

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
