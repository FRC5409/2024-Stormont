package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs;

    public Climber(ClimberIO io) {
        this.io = io;
        this.inputs = new ClimberInputsAutoLogged();
    }

    public Command setPosition(double position) {
        return Commands.runOnce(() -> io.setPosition(position), this);
    }
    
    public Command stop() {
        return Commands.runOnce(io::stop, this);
    }

    public Command setVoltage(double voltage) {
        return Commands.runOnce(() -> io.setVoltage(voltage), this);
    }

    public Command zero() {
        return Commands.runOnce(io::zero, this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.debugPID();
        io.updateInputs(inputs);
        Logger.processInputs("Climber-" + io.getIOName(), inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
