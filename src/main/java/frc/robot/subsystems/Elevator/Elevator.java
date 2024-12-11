package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// 5409: The Chargers
// http://github.com/FRC5409

public class Elevator extends SubsystemBase {

    private ElevatorIO IO;
    private ElevatorInputAutoLogged inputs;

    public Elevator(ElevatorIO IO) {
        this.IO = IO;
        inputs = new ElevatorInputAutoLogged();
    }
    
    public Command ElevatingManual() {
        return Commands.runOnce(()-> IO.setVoltage(6), this);
    }

    public Command LoweringManual() {
        return Commands.runOnce(() -> IO.setVoltage(-6), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> IO.setVoltage(0), this);
    }

    public Command ElevatingPID(double position) {
        return Commands.runOnce(()-> IO.setPosition(position, 0), this);
    }

    public Command LoweringPID(double position) {
        return Commands.runOnce(()-> IO.setPosition(position, 0), this);
    }

    public Command resetPosition(double value) {
        return Commands.runOnce(()-> IO.resetPosition(value), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        IO.updateInputs(inputs);
        IO.debugPID();
        Logger.processInputs("Elevator "+IO.getName(), inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

}
