package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorTalonFX extends SubsystemBase {
    private ElevatorIO io;

    public Command startExtendingTalonFX() {
        return Commands.runOnce(() -> io.setVoltage(6), this);
    }

    public Command startRetracktingTalonFX() {
        return Commands.runOnce(() -> io.setVoltage(-6), this);
    }

    public Command stopTalonFX() {
        return Commands.runOnce(() -> io.setVoltage(0), this);
    }
}