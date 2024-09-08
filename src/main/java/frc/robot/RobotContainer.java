// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoCreator.CustomAutoBuilder;
import frc.robot.AutoCreator.NamedConditions;
import frc.robot.Constants.kController;
import frc.robot.Constants.kDeployment;
import frc.robot.Constants.kDrive;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Deployment;
import frc.robot.subsystems.Drive;

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

    // Joysticks
    private final CommandXboxController m_primaryController;
    // private final CommandXboxController m_secondaryController;

    // Subsystems
    public final Drive sys_drivetrain;
    public final Deployment sys_deployment;

    // Commands
    private final Command cmd_teleopDrive;

    private final SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
            .withDeadband(kDrive.MAX_CHASSIS_SPEED * 0.1)
            .withRotationalDeadband(kDrive.MAX_ROTATION_SPEED * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Shuffleboard
    public final ShuffleboardTab sb_driveteamTab;

    // Autonomous
    private final SendableChooser<Command> sc_autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Joysticks
        m_primaryController = new CommandXboxController(kController.kDriverControllerPort);
        // m_secondaryController = new CommandXboxController(kController.kSecondaryController);
        DriverStation.silenceJoystickConnectionWarning(true);

        // Subsystems
        sys_drivetrain = TunerConstants.DriveTrain;
        sys_deployment = Deployment.getInstance();

        // Commands
        cmd_teleopDrive = sys_drivetrain.applyRequest(() -> {
            return teleopDrive
                    .withVelocityX(-m_primaryController.getLeftY() * kDrive.MAX_CHASSIS_SPEED)
                    .withVelocityY(-m_primaryController.getLeftX() * kDrive.MAX_CHASSIS_SPEED)
                    .withRotationalRate(
                            (m_primaryController.getLeftTriggerAxis() - m_primaryController.getRightTriggerAxis())
                                    * kDrive.MAX_ROTATION_SPEED);
        }).ignoringDisable(true);

        sys_drivetrain.setDefaultCommand(cmd_teleopDrive);

        registerCommands();

        // Shuffleboard
        sb_driveteamTab = Shuffleboard.getTab("Drive team");
        sb_driveteamTab.add("Field", sys_drivetrain.fieldMap).withPosition(3, 0).withSize(7, 4);
        sc_autoChooser = CustomAutoBuilder.buildChooser();

        // Autonomous
        sb_driveteamTab.add("Choose auto", sc_autoChooser)
            .withPosition(0, 1)
            .withSize(3, 1);

        // Configure the trigger bindings
        configureBindings();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("SHOOT", new ShootCommand());

        NamedConditions.registerCondition("NOTE", () -> DriverStation.getAlliance().get() == Alliance.Red);
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

        m_primaryController.x()
            .onTrue(Commands.runOnce(() -> sys_deployment.extendTo(kDeployment.MAX_HEIGHT), sys_deployment));

        m_primaryController.y()
            .onTrue(Commands.runOnce(() -> sys_deployment.extendTo(kDeployment.MIN_HEIGHT), sys_deployment));

        m_primaryController.rightBumper()
                .onTrue(Commands.runOnce(sys_drivetrain::seedFieldRelative, sys_drivetrain));

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
