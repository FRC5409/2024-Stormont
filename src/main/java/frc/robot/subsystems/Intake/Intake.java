package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeInputsAutoLogged();
    }

    public Command runIntake() {
        return Commands.runOnce(() -> io.setVoltage(6), this);
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> io.setVoltage(0), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}