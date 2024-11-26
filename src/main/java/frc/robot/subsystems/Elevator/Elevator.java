package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    //dsd
    private ElevatorIO io;
    private ElevatorInputsAutoLogged inputs;

    public Elevator(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        Shuffleboard.getTab("elevator").addDouble("elevator position", () -> inputs.motorPosition);
    }

    // public Command startExtendingCommand() {
    //     return Commands.runOnce(() -> io.setVoltage(6), this);
    // }

    // public Command startRetracktingCommand() {
    //     return Commands.runOnce(() -> io.setVoltage(-6), this);
    // }

    // public Command stopCommannd() {
    //     return Commands.runOnce(() -> io.setVoltage(0), this);
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }
}
