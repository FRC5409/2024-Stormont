package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private ElevatorIO IO;
    private ElevatorInputAutoLogged inputs;

    public Elevator(ElevatorIO IO) {
        this.IO = IO;
        inputs = new ElevatorInputAutoLogged();
        
        Shuffleboard.getTab("Elevator").addDouble(IO.getName()+"Elevator Position", () -> inputs.elevatorPositionTalon);
    }

    public Command startElevating() {
        return Commands.runOnce(() -> IO.setVoltage(6), this);
    }

    public Command Lowering() {
        return Commands.runOnce(() -> IO.setVoltage(-6), this);
    }

    public Command stopElevating() {
        return Commands.runOnce(() ->IO.setVoltage(0), this);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        IO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
