package frc.robot.commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kDeployment;
import frc.robot.subsystems.Deployment;

/**
 * @author
 */
public class Score extends Command {

    private final Deployment m_deployment;
    private double setpoint;

    public Score(Deployment deployment, double setpoint) {
        m_deployment = deployment;
        this.setpoint = setpoint;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_deployment);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_deployment.setpoint(setpoint);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return Math.abs(m_deployment.getPosition() - kDeployment.setpoints.amp_pos)
        // <= 0.1;
        return false;
    }

}