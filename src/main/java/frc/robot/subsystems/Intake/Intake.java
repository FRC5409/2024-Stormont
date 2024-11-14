package frc.robot.subsystems.Intake;

// 5409: The Chargers
// http://github.com/FRC5409

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private IntakeIO io;
    private IntakeInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeInputsAutoLogged();

        Shuffleboard.getTab("intake").addDouble("intake velocity", () -> inputs.motorSpeed);
    }

    public Command startIntakign() {
        return Commands.runOnce(() -> io.setVoltage(6), this);
    }

    public Command stopIntakign() {
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