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
        m_intake.setVoltage(IntakeConstants.HIGH_VOLTAGE);
        m_indexer.setVoltage(IndexerConstants.HIGH_VOLTAGE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!isSpeedLowered && m_intake.getSensorInterrupted()) {
                m_intake.setVoltage(IntakeConstants.LOW_VOLTAGE);
                m_indexer.setVoltage(IndexerConstants.LOW_VOLTAGE);

                isSpeedLowered = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setVoltage(0);
        m_indexer.setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return m_indexer.getSensorInterrupted();
        return false;
    }

}