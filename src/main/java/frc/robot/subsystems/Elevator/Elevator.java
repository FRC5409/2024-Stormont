package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kElevator;

public class Elevator extends SubsystemBase {
    private ElevatorInputsAutoLogged inputs;
    private ElevatorIO io;

    private final ShuffleboardTab sb_elevator;

    public Elevator(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        sb_elevator = Shuffleboard.getTab("Elevator" + io.getMotorName());
        sb_elevator.addDouble("kP", () -> io.getP());
        sb_elevator.addDouble("kI", () -> io.getI());
        sb_elevator.addDouble("kD", () -> io.getD());
        sb_elevator.addDouble("Pos", () -> io.getPos());
    }

    public Command moveSeconds(boolean up, double seconds) {
        return Commands.startEnd(
                () -> io.setVoltage(up ? kElevator.VOLTAGE : -kElevator.VOLTAGE),
                () -> io.setVoltage(0),
                this)
            .withTimeout(seconds);
    }

    public Command moveToUp(boolean up) {
        return Commands.runOnce(() -> io.setPosition(up ? 1 : 0, 0), this);
    }

    public Command setSetpoint(double setpoint, int slot) {
        return Commands.runOnce(() -> io.setPosition(setpoint, 0), this);
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