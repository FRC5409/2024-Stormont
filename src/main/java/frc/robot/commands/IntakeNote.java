package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * @author 
 */
public class IntakeNote extends Command {

    private boolean isSpeedLowered = false;

    private final Intake m_intake;
    private final Indexer m_indexer;

    public IntakeNote() {
        m_intake = Intake.getInstance();
        m_indexer = Indexer.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intake, m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.setSpeed(IntakeConstants.HIGH_SPEED);
        m_indexer.setSpeed(IndexerConstants.HIGH_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!isSpeedLowered && m_indexer.getSensorValue()) {
                m_intake.setSpeed(IntakeConstants.LOW_SPEED);
                m_indexer.setSpeed(IndexerConstants.LOW_SPEED);

                isSpeedLowered = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}