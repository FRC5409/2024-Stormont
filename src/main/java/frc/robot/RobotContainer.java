// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoCreator.CustomAutoBuilder;
import frc.robot.AutoCreator.NamedConditions;
import frc.robot.Constants.kAuto;
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

        sb_driveteamTab.add("Note", Commands.runOnce(() -> {Robot.hasNote = true;}));

        // Configure the trigger bindings
        configureBindings();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("SHOOT", new ShootCommand().alongWith(Commands.runOnce(() -> {Robot.hasNote = false;})));
        NamedConditions.registerCondition("NOTE", () -> Robot.hasNote);

        String[] condtions = new String[Robot.notes.length];
        String[] returnPaths = new String[Robot.notes.length];

        Command[] commands = new Command[Robot.notes.length];

        for (int i = 0; i < Robot.notes.length; i++) {
            int noteNum = i + 1;

            String shootingPos = noteNum < 3 ? "A" : "B";

            condtions[i] = "" + noteNum + "_PRESENT";
            String grabbedNoteCommand = "GRABBED_NOTE" + noteNum;

            final int innerI = i; // ???
            NamedConditions.registerCondition(condtions[i], () -> Robot.notes[innerI]);
            NamedCommands.registerCommand(grabbedNoteCommand, Commands.runOnce(() -> {Robot.notes[innerI] = false;}));

            returnPaths[innerI] = "MNOTE" + noteNum + "_TO_SHOOT" + shootingPos;
        }

        String[] newCondtions = new String[Robot.notes.length];
        for (int i = 1; i < Robot.notes.length; i++) {
            newCondtions[i - 1] = condtions[i];
        }
        newCondtions[Robot.notes.length - 1] = "ALWAYS_TRUE"; // Should be a condtion that never gets checked

        for (int i = 0; i < newCondtions.length; i++) {
            System.out.println(newCondtions[i]);
        }

        NamedCommands.registerCommand("RETURN_PATH", CustomAutoBuilder.buildCaseCommand(newCondtions, returnPaths, Commands.none()));

        for (int i = 0; i < Robot.notes.length; i++) {
            int noteNum = i + 1;

            String shootingPos = noteNum < 3 ? "A" : "B";

            String notePaths = "SHOOT" + shootingPos + "_TO_MNOTE" + noteNum;

            commands[i] = new SequentialCommandGroup(
                CustomAutoBuilder.buildPathCommand(notePaths).until(NamedConditions.getCondition("NOTE")),

                new ConditionalCommand(
                    Commands.none(), 
                    new ParallelDeadlineGroup(
                        Commands.idle().until(NamedConditions.getCondition("NOTE")),
                        CustomAutoBuilder.buildPathCommand("DOWN_NOTES")
                    ).andThen(
                        Commands.runOnce(() -> {
                            int closestNote = 0;
                            double distance = Math.abs(sys_drivetrain.getRobotPose().getY() - kAuto.NOTES[3].getY());
                        
                            // Find the closest note
                            for (int j = 4; j < kAuto.NOTES.length; j++) {
                                double newDistance = Math.abs(sys_drivetrain.getRobotPose().getY() - kAuto.NOTES[j].getY());
                                if (newDistance < distance) {
                                    closestNote = j - 3;
                                    distance = newDistance;
                                }
                            }
                        
                            final int finalClosestNote = closestNote;
                            for (int j = 0; j < finalClosestNote; j++) {
                                Robot.notes[j] = false;
                            }
                        })
                    ),
                    NamedConditions.getCondition("NOTE")
                ),

                NamedCommands.getCommand("GRABBED_NOTE" + noteNum),
                NamedCommands.getCommand("RETURN_PATH"),
                NamedCommands.getCommand("SHOOT")
            );
        }


        NamedCommands.registerCommand(
            "PICK_PATH",
            CustomAutoBuilder.buildCaseCommand(condtions, commands, CustomAutoBuilder.buildPathCommand("SHOOTB_TO_MIDLINE"))
        );
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

        m_primaryController.leftBumper()
            .onTrue(Commands.runOnce(() -> {Robot.hasNote = true;}).ignoringDisable(true));

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
