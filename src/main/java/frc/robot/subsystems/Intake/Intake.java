package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeInputsAutoLogged;

public class Intake extends SubsystemBase {
    private IntakeInputsAutoLogged inputs;
    private IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public Command start() {
        return Commands.runOnce(() -> io.setVoltage(6), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> io.setVoltage(0), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

}