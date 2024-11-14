package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private ElevatorInputsAutoLogged inputs;
    private ElevatorIO io;

    private Elevator(ElevatorIO io) {
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

}