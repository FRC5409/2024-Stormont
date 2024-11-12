package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private IntakeIO IO;
    private IntakeInputAutoLogged inputs;

    public Intake(IntakeIO IO) {
        this.IO = IO;
        inputs = new IntakeInputAutoLogged();
    }

    //intake, startIntaking, start
    public Command startIntaking() {
        return Commands.runOnce(() -> IO.setVoltage(6), this);
    }

    //stopIntaking, stop
    public Command stopIntaking() {
        return Commands.runOnce(() -> IO.setVoltage(0), this);
    }

    //stopIntaking, stop

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        IO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
