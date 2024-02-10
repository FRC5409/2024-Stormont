package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * @author Logan Dhillon
 */
public class IntakeEject extends Command {
    private final Intake intake;
    private final Indexer indexer;

    public IntakeEject() {
        intake = Intake.getInstance();
        indexer = Indexer.getInstance();

        addRequirements(intake, indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setVoltage(-IntakeConstants.HIGH_VOLTAGE);
        indexer.setVoltage(-IndexerConstants.HIGH_VOLTAGE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.setVoltage(0);
        // give the intake some time to push the note all the way out of the subsystem
        if (!interrupted) Commands.waitSeconds(0.5);
        intake.setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return intake.getSensorValue();
    }
}