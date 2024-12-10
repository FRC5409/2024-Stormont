// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.elevator.ElevatorIOTalon;

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


    // Subsystem
    // private final Intake sys_intake;
    private final Elevator sys_elevatorMAX;
    private final Elevator sys_elevatorTalon;

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
        DriverStation.silenceJoystickConnectionWarning(true);

        // Subsystems
        switch (Constants.getMode()) {
            case REAL -> {
                // sys_intake = new Intake(new IntakeIOSparkMax(0));
                sys_elevatorMAX = new Elevator(new ElevatorIOSparkMax(6));
                sys_elevatorTalon = new Elevator(new ElevatorIOTalon(8));
            }
            case REPLAY -> {
                // sys_intake = new Intake(new IntakeIO() {
                // });
                sys_elevatorMAX = new Elevator(new ElevatorIO() {
                });
                sys_elevatorTalon = new Elevator(new ElevatorIO() {
                });
            }
            case SIM -> {
                // sys_intake = new Intake(new IntakeIOSim());
                sys_elevatorMAX = new Elevator(new ElevatorIOSim());
                sys_elevatorTalon = new Elevator(new ElevatorIOSim());
            }
            default -> throw new IllegalArgumentException("Couldn't find a mode to init subsystems to...");
        }


        // Commands
       

        // Shuffleboard
        sb_driveteamTab = Shuffleboard.getTab("Drive team");

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
        // m_primaryController.a().onTrue(sys_intake.start()).onFalse(sys_intake.stop());

        m_primaryController.a().onTrue(sys_elevatorMAX.moveToUp(true));
        m_primaryController.y().onTrue(sys_elevatorMAX.moveToUp(false ));

        m_primaryController.x().onTrue(sys_elevatorTalon.moveToUp(true ));
        m_primaryController.b().onTrue(sys_elevatorTalon.moveToUp(false));

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
