package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * @author 
 */
public class StoreNote extends Command {

    private final Intake m_intake;

    public StoreNote() {
        m_intake = Intake.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.setOuterRoller(4.0, true);
        m_intake.setInnerRoller(4.0, true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.stopAllMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_intake.getSensorValue();
    }

}