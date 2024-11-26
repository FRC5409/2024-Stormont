// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kController;
import frc.robot.Constants.kDrive;
import frc.robot.subsystems.Drive.Drive;

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
    private final Drive sys_drivetrain;

    // Commands

    // Shuffleboard
    public final ShuffleboardTab sb_driveteamTab;

    // Autonomous

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Joysticks
        m_primaryController = new CommandXboxController(kController.kDriverControllerPort);
        // m_secondaryController = new CommandXboxController(kController.kSecondaryController);
        // DriverStation.silenceJoystickConnectionWarning(true);

        // Subsystems
        // switch (Constants.getMode()) {
        //     case REAL -> {
        //         sys_deployment = Deployment.createInstance(new DeploymentIOSparkMax(kDeployment.DEPLOYMENT_ID));
        //         sys_vision = Vision.createInstance(new VisionIOLimelight(kVision.LIMELIGHT_OFFSET));
        //     }
        //     case REPLAY -> {
        //         sys_deployment = Deployment.createInstance(new DeploymentIO() {});
        //         sys_vision = Vision.createInstance(new VisionIO() {});
        //     }
        //     case SIM -> {
        //         sys_deployment = Deployment.createInstance(new DeploymentIOSim());
        //         sys_vision = Vision.createInstance(new VisionIO() {});
        //     }
        //     case DEMO -> {
        //         // TODO: Finish this
        //         sys_deployment = null;
        //         sys_vision = null;
        //     }
        //     default -> {
        //         throw new IllegalArgumentException("Couldn't find a mode to init subsystems to...");
        //     }
        // }

        sys_drivetrain = Drive.getInstance();

        // Commands
        sys_drivetrain.setDefaultCommand(
            sys_drivetrain.telopDrive(
                () ->  m_primaryController.getLeftX() * kDrive.MAX_CHASSIS_SPEED,
                () ->  m_primaryController.getLeftY() * kDrive.MAX_CHASSIS_SPEED,
                () -> calculateRotationOutput(m_primaryController.getLeftTriggerAxis() - m_primaryController.getRightTriggerAxis())
            )
        );

        // Shuffleboard
        sb_driveteamTab = Shuffleboard.getTab("Drive team");
        sb_driveteamTab.add("Field", sys_drivetrain.fieldMap).withPosition(3, 0).withSize(7, 4);

        // Configure the trigger bindings
        configureBindings();
    }

    public double calculateRotationOutput(double input) {
        double output = 0;
        if (input > 0) {
            output = (kDrive.MAX_ROTATION_SPEED - 2) * Math.pow(input, 3) + input + Math.sqrt(input);
        } else if (input < 0) {
            output = -((kDrive.MAX_ROTATION_SPEED - 2) * Math.pow(-input, 3) - input + Math.sqrt(-input));
        }
        
        return MathUtil.clamp(output, -kDrive.MAX_ROTATION_SPEED, kDrive.MAX_ROTATION_SPEED);
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
        // m_primaryController.a().onTrue(sys_deployment.deploy()).onFalse(sys_deployment.retract());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
