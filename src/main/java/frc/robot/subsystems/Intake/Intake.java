package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private IntakeIO io;
    private IntakeInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeInputsAutoLogged();        
    }

    // Spin the intake
    public Command startIntaking() {
        return Commands.runOnce(() -> io.setVoltage(6), this);
    }

    // Stop spinning the intake
    public Command stopIntaking() {
        return Commands.runOnce(() -> io.setVoltage(0), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}