// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kController;
import frc.robot.Constants.kClimber.kSpark;
import frc.robot.Constants.kClimber.kTalon;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOSparkMax;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;

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
    private final CommandXboxController m_secondaryController;

    // Subsystems
    private final Climber sys_sparkClimber;
    private final Climber sys_talonClimber;

    // Commands

    // Shuffleboard

    // Autonomous

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Joysticks
        m_primaryController = new CommandXboxController(kController.kDriverControllerPort);
        m_secondaryController = new CommandXboxController(kController.kSecondaryController);
        DriverStation.silenceJoystickConnectionWarning(true);

        // Subsystems
        switch (Constants.getMode()) {
            case REAL_NO_LOG -> {
                sys_sparkClimber = new Climber(new ClimberIOSparkMax(kSpark.ID));
                sys_talonClimber = new Climber(new ClimberIOTalonFX(kTalon.ID));
            }
            case REAL -> {
                sys_sparkClimber = new Climber(new ClimberIOSparkMax(kSpark.ID));
                sys_talonClimber = new Climber(new ClimberIOTalonFX(kTalon.ID));
            }
            case REPLAY -> {
                sys_sparkClimber = new Climber(new ClimberIO() {});
                sys_talonClimber = new Climber(new ClimberIO() {});
            }
            case SIM -> {
                sys_sparkClimber = null;
                sys_talonClimber = null;
            }
            default -> throw new IllegalArgumentException("Couldn't find a mode to init subsystems to...");
        }

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
        // Button Bindings here

        // TODO: edit these numbers for PID tuning
        m_primaryController.povUp()
            .onTrue(sys_sparkClimber.setPosition(0))
            .onFalse(sys_sparkClimber.stop());

        m_primaryController.povDown()
            .onTrue(sys_sparkClimber.setPosition(0))
            .onFalse(sys_sparkClimber.stop());

        m_primaryController.y()
            .onTrue(sys_talonClimber.setPosition(0))
            .onFalse(sys_talonClimber.stop());

        m_primaryController.a()
            .onTrue(sys_talonClimber.setPosition(0))
            .onFalse(sys_talonClimber.stop());
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
