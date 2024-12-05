package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// 5409: The Chargers
// http://github.com/FRC5409

public class Elevator extends SubsystemBase {

    private ElevatorIO IO;
    private ElevatorInputAutoLogged inputs;

    private ShuffleboardTab sb_elevator;

    public Elevator(ElevatorIO IO) {
        this.IO = IO;
        inputs = new ElevatorInputAutoLogged();

        sb_elevator = Shuffleboard.getTab("Elevator");
        sb_elevator.addDouble(IO.getName() + " Position",
                () -> inputs.elevatorPosition);
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

    public Command ElevatingPID() {
        return Commands.runOnce(()-> IO.setPosition(10), this);
    }

    public Command LoweringPID() {
        return Commands.runOnce(()-> IO.setPosition(0), this);
    }

    public Command resetPosition() {
        return Commands.runOnce(()-> IO.resetPosition(0), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        IO.updateInputs(inputs);
        Logger.processInputs("Elevator "+IO.getName(), inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

}
