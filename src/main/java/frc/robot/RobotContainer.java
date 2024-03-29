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
import frc.robot.Constants.kControllers;
import frc.robot.Constants.kDeployment;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kRobot;
import frc.robot.Constants.kWaypoints;
import frc.robot.Constants.kDrive.kAutoAlign;
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
    private final SendableChooser<Command> sc_autoChooser;
    private final SendableChooser<Boolean> sc_alliance;
    private final GenericEntry sb_autoDelay;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Joysticks
        m_primaryController = new CommandXboxController(kControllers.PRIMARY_CONTROLLER);
        m_secondaryController = new CommandXboxController(kControllers.SECONDARY_CONTROLLER);
        DriverStation.silenceJoystickConnectionWarning(true);

        // Subsystems
        sys_climber = new Climber();
        sys_intake = Intake.getInstance();
        sys_indexer = Indexer.getInstance();
        sys_deployment = new Deployment();
        sys_cartridge = Cartridge.getInstance();
        sys_photonvision = PhotonVision.getInstance();

        // Subsystems
        sys_drivetrain =
                kRobot.IS_BETA_ROBOT
                        ? TunerConstantsBeta.DriveTrain
                        : TunerConstantsComp.DriveTrain;

        // Commands
        cmd_teleopDrive =
                sys_drivetrain.drive(
                        () -> -m_primaryController.getLeftY() * kDrive.MAX_DRIVE_VELOCIY,
                        () -> -m_primaryController.getLeftX() * kDrive.MAX_DRIVE_VELOCIY,
                        () ->
                                (m_primaryController.getLeftTriggerAxis()
                                                - m_primaryController.getRightTriggerAxis())
                                        * kDrive.MAX_TURN_ANGULAR_VELOCITY);

        cmd_intakeToSensor = 
            Commands.runOnce(() -> {
                sys_intake.setVoltage(kIntake.VOLTAGE);
                sys_indexer.setVoltage(kIndexer.VOLTAGE);
            }, 
            sys_intake, sys_indexer)
            .andThen(Commands.waitUntil(() -> sys_intake.checkIR()))
            .andThen(Commands.runOnce(() -> {
                sys_intake.setVoltage(0);
                sys_indexer.setVoltage(0);
            }, 
            sys_intake, sys_indexer));

        sys_drivetrain.setDefaultCommand(cmd_teleopDrive);

        registerPathplannerCommands();

        // Shuffleboard
        sb_driveteamTab = Shuffleboard.getTab("Drive team");
        sc_autoChooser = AutoBuilder.buildAutoChooser();

        sc_alliance = new SendableChooser<>();
        sc_alliance.addOption("Blue", false);
        sc_alliance.addOption("Red", true);
        sc_alliance.setDefaultOption("Blue", false);

        sb_autoDelay = sb_driveteamTab.add("Auto delay", 0.0).getEntry();

        // Autonomous
        sb_driveteamTab.add("Choose auto", sc_autoChooser).withPosition(0, 0).withSize(3, 1);
        sb_driveteamTab.add("Alliance", sc_alliance).withPosition(0, 1).withSize(3, 1);


        // Configure the trigger bindings
        configureBindings();

        new Trigger(() -> sys_indexer.checkIR())
                .and(DriverStation::isTeleop)
                .onTrue(
                        new BringNoteToCartridge(sys_cartridge, sys_indexer)
                                .onlyIf(DriverStation::isTeleop))
                .onTrue(
                        Commands.runOnce(
                                        () ->
                                                m_primaryController
                                                        .getHID()
                                                        .setRumble(RumbleType.kBothRumble, 0.3))
                                .andThen(new WaitCommand(0.75))
                                .andThen(
                                        Commands.runOnce(
                                                () ->
                                                        m_primaryController
                                                                .getHID()
                                                                .setRumble(
                                                                        RumbleType.kBothRumble,
                                                                        0.0))));
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
        m_primaryController
                .rightBumper()
                .onTrue(Commands.runOnce(sys_drivetrain::seedFieldRelative, sys_drivetrain));

        // score command
        m_primaryController.a().onTrue(new ScoreNote(sys_deployment, sys_cartridge));

        // Intake note command
        m_primaryController
                .x()
                .whileTrue(
                        Commands.race(
                                Commands.startEnd(
                                        () -> {
                                            sys_intake.setVoltage(kIntake.VOLTAGE);
                                            sys_indexer.setVoltage(kIndexer.VOLTAGE);
                                        },
                                        () -> {
                                            sys_intake.setVoltage(0);
                                            sys_indexer.setVoltage(0);
                                        },
                                        sys_intake,
                                        sys_indexer),
                                Commands.waitUntil(() -> sys_indexer.checkIR())));

        // Eject note command
        m_primaryController
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    sys_cartridge.setVoltage(-kCartridge.VOLTAGE);
                                    sys_intake.setVoltage(-12);
                                },
                                sys_cartridge,
                                sys_intake))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    sys_cartridge.setVoltage(0);
                                    sys_intake.setVoltage(0);
                                },
                                sys_cartridge,
                                sys_intake));

        m_primaryController.y()
                                .onTrue(cmd_intakeToSensor)
                                .onFalse(Commands.runOnce(() -> sys_intake.setVoltage(0), sys_intake));

        m_primaryController.start().onTrue(new BringNoteToCartridge(sys_cartridge, sys_indexer));

        m_primaryController
                .leftBumper()
                .onTrue(
                        new BringNoteToCartridge(sys_cartridge, sys_indexer)
                                .andThen(
                                        Commands.runOnce(
                                                () ->
                                                        sys_deployment.setPosition(
                                                                kDeployment
                                                                        .kSetpoints
                                                                        .AMP_POSITION),
                                                sys_deployment)))
                // .whileTrue(new AlignToPose(sys_drivetrain.getAmpWaypoint(), sys_drivetrain));
                .whileTrue(
                        new AlignToPose(
                                () -> sys_drivetrain.getAmpWaypoint(isRed()),
                                sys_drivetrain,
                                true,
                                kAutoAlign.REACHED_POSITION_TIMEOUT_SLOW,
                                kAutoAlign.REACHED_POSITION_TOLERANCE_ClOSE,
                                isRed()));

        // Secondary Controller
        // *************************************************************************************************************

        // Climber setpoint high fast
        m_secondaryController
                .povUp()
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        sys_climber.setPosition(
                                                Constants.kClimber.HIGH,
                                                Constants.kClimber.KFAST_SLOT),
                                sys_climber));

        // Climber setpoint high slow
        m_secondaryController
                .povLeft()
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        sys_climber.setPosition(
                                                Constants.kClimber.HIGH,
                                                Constants.kClimber.KSLOW_SLOT),
                                sys_climber));

        // Climber setpoint low fast
        m_secondaryController
                .povDown()
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        sys_climber.setPosition(
                                                Constants.kClimber.LOW,
                                                Constants.kClimber.KFAST_SLOT),
                                sys_climber));

        // Climber manual down
        m_secondaryController
                .leftBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> sys_climber.setVoltage(Constants.kClimber.VOLTAGE),
                                sys_climber))
                .onFalse(Commands.runOnce(() -> sys_climber.setVoltage(0), sys_climber));

        // Climber manual up
        m_secondaryController
                .rightBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> sys_climber.setVoltage(-Constants.kClimber.VOLTAGE),
                                sys_climber))
                .onFalse(Commands.runOnce(() -> sys_climber.setVoltage(0), sys_climber));

        // turns off climber pid
        m_secondaryController
                .start()
                .onTrue(Commands.runOnce(() -> sys_climber.setVoltage(0), sys_climber));

        // Bring note from indexer to cartridge, when stuck
        m_secondaryController.back().onTrue(new BringNoteToCartridge(sys_cartridge, sys_indexer));

        // Climb, extend and score, endgame sequence
        m_secondaryController.y().onTrue(new ScoreTrap(sys_deployment, sys_cartridge, sys_climber));

        m_secondaryController
                .x()
                .whileTrue(
                        new AlignToPose(
                                        () -> {
                                            double trapRotation = sys_drivetrain.getTrapRotation(1);
                                            System.out.println(trapRotation);
                                            return sys_photonvision.getNearestTagPoseWithOffset(
                                                        sys_drivetrain,
                                                        kWaypoints.TRAP_DISTANT_OFFSET,
                                                        trapRotation);
                                        },
                                        sys_drivetrain,
                                        false,
                                        kAutoAlign.REACHED_POSITION_TIMEOUT_FAST,
                                        kAutoAlign.REACHED_POSITION_TOLERANCE,
                                        isRed())
                                .andThen(
                                        new AlignToPose(
                                                () -> {
                                            double trapRotation = sys_drivetrain.getTrapRotation(1);
                                            System.out.println(trapRotation);
                                            return sys_photonvision.getNearestTagPoseWithOffset(
                                                        sys_drivetrain,
                                                        kWaypoints.TRAP_OFFSET,
                                                        trapRotation);
                                        },
                                                sys_drivetrain,
                                                true,
                                                kAutoAlign.REACHED_POSITION_TIMEOUT_SLOW,
                                                kAutoAlign.REACHED_POSITION_TOLERANCE_ClOSE,
                                                isRed())));

        m_secondaryController
                .b()
                .whileTrue(
                        new AlignToPose(
                                        () -> {
                                            double trapRotation = sys_drivetrain.getTrapRotation(2);
                                            System.out.println(trapRotation);
                                            return sys_photonvision.getNearestTagPoseWithOffset(
                                                        sys_drivetrain,
                                                        kWaypoints.TRAP_DISTANT_OFFSET,
                                                        trapRotation);
                                        },
                                        sys_drivetrain,
                                        false,
                                        kAutoAlign.REACHED_POSITION_TIMEOUT_FAST,
                                        kAutoAlign.REACHED_POSITION_TOLERANCE,
                                        isRed())
                                .andThen(
                                        new AlignToPose(
                                                () -> {
                                            double trapRotation = sys_drivetrain.getTrapRotation(2);
                                            System.out.println(trapRotation);
                                            return sys_photonvision.getNearestTagPoseWithOffset(
                                                        sys_drivetrain,
                                                        kWaypoints.TRAP_OFFSET,
                                                        trapRotation);
                                        },
                                                sys_drivetrain,
                                                true,
                                                kAutoAlign.REACHED_POSITION_TIMEOUT_SLOW,
                                                kAutoAlign.REACHED_POSITION_TOLERANCE_ClOSE,
                                                isRed())));

        m_secondaryController
                .a()
                .whileTrue(
                        new AlignToPose(
                                        () -> {
                                            double trapRotation = sys_drivetrain.getTrapRotation(3);
                                            System.out.println(trapRotation);
                                            return sys_photonvision.getNearestTagPoseWithOffset(
                                                        sys_drivetrain,
                                                        kWaypoints.TRAP_DISTANT_OFFSET,
                                                        trapRotation);
                                        },
                                        sys_drivetrain,
                                        false,
                                        kAutoAlign.REACHED_POSITION_TIMEOUT_FAST,
                                        kAutoAlign.REACHED_POSITION_TOLERANCE,
                                        isRed())
                                .andThen(
                                        new AlignToPose(
                                                () -> {
                                            double trapRotation = sys_drivetrain.getTrapRotation(3);
                                            System.out.println(trapRotation);
                                            return sys_photonvision.getNearestTagPoseWithOffset(
                                                        sys_drivetrain,
                                                        kWaypoints.TRAP_OFFSET,
                                                        trapRotation);
                                        },
                                                sys_drivetrain,
                                                true,
                                                kAutoAlign.REACHED_POSITION_TIMEOUT_SLOW,
                                                kAutoAlign.REACHED_POSITION_TOLERANCE_ClOSE,
                                                isRed())));
    }

    public void registerPathplannerCommands() {

        NamedCommands.registerCommand(
                "IntakeFromFloor",
                Commands.run(
                                () -> {
                                    sys_intake.setVoltage(kIntake.VOLTAGE);
                                    sys_indexer.setVoltage(kIndexer.VOLTAGE);
                                },
                                sys_intake,
                                sys_indexer)
                        .until(sys_indexer::checkIR));

        NamedCommands.registerCommand(
                "BringNoteToCartridge", new BringNoteToCartridge(sys_cartridge, sys_indexer));
        NamedCommands.registerCommand(
                "ScoreNote", new ScoreNote(sys_deployment, sys_cartridge).withTimeout(1));
        NamedCommands.registerCommand("offsetFieldRelativeForward", Commands.runOnce(() -> sys_drivetrain.offsetFieldRelative(0, isRed()), sys_drivetrain));
        NamedCommands.registerCommand("offsetFieldRelativeLeft", Commands.runOnce(() -> sys_drivetrain.offsetFieldRelative(Math.toRadians(90), isRed()), sys_drivetrain));
        NamedCommands.registerCommand("offsetFieldRelativeRight", Commands.runOnce(() -> sys_drivetrain.offsetFieldRelative(Math.toRadians(-90), isRed()), sys_drivetrain));
        NamedCommands.registerCommand("offsetFieldRelativeBackward", Commands.runOnce(() -> sys_drivetrain.offsetFieldRelative(Math.toRadians(-180), isRed()), sys_drivetrain));
        NamedCommands.registerCommand("EjectNote", Commands.runOnce(() -> sys_cartridge.setVoltage(-kCartridge.VOLTAGE), sys_cartridge).withTimeout(1));
        // .alongWith(new AlignToPose(() -> sys_drivetrain.getAmpWaypoint(),
        // sys_drivetrain)));

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
        return Commands.waitSeconds(sb_autoDelay.getDouble(0.0)).andThen(
        sc_autoChooser.getSelected());
    }
}
