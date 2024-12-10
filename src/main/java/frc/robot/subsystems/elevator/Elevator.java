package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private ElevatorInputsAutoLogged inputs;
    private ElevatorIO io;

    private ShuffleboardTab sb;
    private GenericEntry kP;
    private GenericEntry kI;
    private GenericEntry kD;


    public Elevator(ElevatorIO io) {
        this.sb = Shuffleboard.getTab("Elevator " + io.getClass().getName());
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        kP = sb.add("kP", 0).getEntry(); 
        kI = sb.add("kI", 0).getEntry();
        kD = sb.add("kD", 0).getEntry();
        sb.addDouble("Position", io::getPosition);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        io.setPID(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0));
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

    public Command moveFor(boolean up, double seconds) {
        return Commands.startEnd(
                () -> io.setVoltage(up ? 6 : -6),
                () -> io.setVoltage(0),
                this).withTimeout(seconds);
    }

    public Command moveToUp(boolean up) {
        return Commands.runOnce(() -> io.setPosition(up ? 60 : 10, 0), this);
    }

}