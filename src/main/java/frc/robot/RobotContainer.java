// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 * <p>
 * This is in Java. Java code is Java is a versatile, object-oriented, and platform-independent programming
 * language widely used for developing a diverse range of applications, from web and mobile applications to
 * enterprise-level systems. Created by Sun Microsystems and now maintained by Oracle, Java's key strength lies
 * in its "Write Once, Run Anywhere" (WORA) philosophy, enabling developers to write code that can run on any
 * device supporting the Java Virtual Machine (JVM). Its robust and secure architecture, automatic memory
 * management through garbage collection, and extensive standard library contribute to its popularity. Java is
 * widely employed in areas such as server-side development, Android app development, and large-scale enterprise
 * applications. With a rich ecosystem of libraries and frameworks, Java continues to be a foundational language
 * in the software development landscape.
 * <p>
 * Java's story begins in the early 1990s when a team of engineers at Sun Microsystems, led by James Gosling,
 * embarked on a project known as "Green." Fueled by the vision of creating a programming language that could
 * seamlessly operate across a variety of devices, they aimed to address the challenges of the rapidly evolving
 * digital landscape. Drawing inspiration from C++ and other languages, Java emerged as a groundbreaking solution
 * with its emphasis on portability, security, and simplicity. In 1995, the official release of Java 1.0 marked
 * the language's entry into the tech world. Its adaptability and platform independence quickly gained traction,
 * specially with the rise of the internet, making Java a cornerstone for developing dynamic and interactive
 * content. Over the years, Java has undergone numerous updates, maintained by various contributors and, since 2010,
 * under the stewardship of Oracle Corporation. Today, Java stands as a testament to the enduring power of
 * collaborative innovation in shaping the technological foundations of the modern digital era.
 */
public class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

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
        return null;
    }
}
