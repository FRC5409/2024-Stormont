package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSparkMax extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorInputsAutoLogged inputs;

    public ElevatorSparkMax(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        Shuffleboard.getTab("elevator").addDouble("elevator position", () -> inputs.motorPosition);
    }

    public Command startExtendingSparkMax() {
        return Commands.runOnce(() -> io.setVoltage(6), this);
    }

    public Command startRetracktingSparkMax() {
        return Commands.runOnce(() -> io.setVoltage(-6), this);
    }

    public Command stopSparkMax() {
        return Commands.runOnce(() -> io.setVoltage(0), this);
    }
}
