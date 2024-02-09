// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Climber sys_climber;

  private final CommandXboxController joystickMain;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    sys_climber = new Climber();
    joystickMain = new CommandXboxController(0);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // manual movement up
    joystickMain.povUp()
    .whileTrue(Commands.runOnce(() -> sys_climber.manualExtend(Constants.kClimber.voltage), sys_climber))
    .whileFalse(Commands.runOnce(() -> sys_climber.manualExtend(0), sys_climber));

    // manual movement down
    joystickMain.povDown()
    .whileTrue(Commands.runOnce(() -> sys_climber.manualExtend(-Constants.kClimber.voltage), sys_climber))
    .whileFalse(Commands.runOnce(() -> sys_climber.manualExtend(0), sys_climber));

    // setpoint high
    joystickMain.a()
    .onTrue(Commands.runOnce(() -> sys_climber.setpoint(Constants.kClimber.high), sys_climber));

    // while holding setpoint goes high, when let go setpoint goes low
    joystickMain.x()
    .whileTrue(Commands.runOnce(() -> sys_climber.setpoint(Constants.kClimber.high), sys_climber))
    .whileFalse(Commands.runOnce(() -> sys_climber.setpoint(Constants.kClimber.low), sys_climber));
    
    // zero encoder
    joystickMain.start()
    .onTrue(Commands.runOnce(() -> sys_climber.zeroEncoderIR(), sys_climber));


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
