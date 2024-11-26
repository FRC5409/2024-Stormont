package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private ElevatorInputsAutoLogged inputs;
    private ElevatorIO io;

    public Elevator(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();
    }

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

    public Command moveFor(boolean up, double seconds) {
        return Commands.startEnd(
                () -> io.setVoltage(up ? 6 : -6),
                () -> io.setVoltage(0),
                this).withTimeout(seconds);
    }

    public Command moveToUp(boolean up) {
        return Commands.runOnce(() -> io.setPosition(up ? 1 : 0, 0), this);
    }

}