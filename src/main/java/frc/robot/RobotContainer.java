// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kController;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFx;


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
    //private final Intake sys_intake;
    private final Elevator sys_elevator_Neo;
    private final Elevator sys_elevator_Falcon;

    // Commands
    
    //private final SendableChooser<Command> sb_autoChooser;

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
            case REAL -> {
                // sys_intake = new Intake(new IntakeIOSparkMax(0));
                sys_elevator_Neo = new Elevator(new ElevatorIOSparkMax(6));
                sys_elevator_Falcon = new Elevator(new ElevatorIOTalonFx(8));
            }
            case REPLAY -> {
                // sys_intake = new Intake(new IntakeIO() {});
                sys_elevator_Neo = new Elevator(new ElevatorIO() {});
                sys_elevator_Falcon = new Elevator(new ElevatorIO() {});
            }
            case SIM -> {
                // sys_intake = new Intake(new IntakeIOSim());
                sys_elevator_Neo = new Elevator(new ElevatorIOSim(DCMotor.getNEO(1), "Neo"));
                sys_elevator_Falcon = new Elevator(new ElevatorIOSim(DCMotor.getFalcon500(1), "Talon"));
            }
            default -> throw new IllegalArgumentException("Couldn't find a mode to init subsystems to...");
        }


        // Commands
       

        // Shuffleboard

        // sb_autoChooser = AutoBuilder.buildAutoChooser();

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
        // m_primaryController.x()
        //     .onTrue(sys_intake.startIntaking())
        //     .onFalse(sys_intake.stopIntaking());
         m_primaryController.povUp()
            .onTrue(sys_elevator_Neo.extend(32))
            .onFalse(sys_elevator_Neo.stop());
         m_primaryController.povDown()
             .onTrue(sys_elevator_Neo.retract(10))
             .onFalse(sys_elevator_Neo.stop());
         m_primaryController.povRight()
            .onTrue(sys_elevator_Falcon.extend(47))
            .onFalse(sys_elevator_Falcon.stop());
         m_primaryController.povLeft()
             .onTrue(sys_elevator_Falcon.retract(10))
             .onFalse(sys_elevator_Falcon.stop());
         m_primaryController.a()
             .onTrue(sys_elevator_Neo.m_extend())
             .onFalse(sys_elevator_Neo.stop());
         m_primaryController.b()
             .onTrue(sys_elevator_Falcon.m_extend())
             .onFalse(sys_elevator_Falcon.stop());
         m_primaryController.x()
             .onTrue(sys_elevator_Neo.m_retract())
             .onFalse(sys_elevator_Neo.stop());
         m_primaryController.y()
             .onTrue(sys_elevator_Falcon.m_retract())
             .onFalse(sys_elevator_Falcon.stop());
          m_primaryController.rightBumper()
              .onTrue(sys_elevator_Neo.reset().alongWith(sys_elevator_Falcon.reset()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //return sb_autoChooser.getSelected();
        return null;
    }
}
