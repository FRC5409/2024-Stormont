package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    
    private ElevatorIO io;
    private ElevatorInputsAutoLogged inputs;
    // private Double kP;
    // private Double kI;
    // private Double kD;


    public Elevator(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();
        System.out.println(io.getName());
        Shuffleboard.getTab("Elevator").addDouble("NeoMotorPosition"+ io.getName(), () -> inputs.motorPositionNEO);
    }

    // Get subsystem

    //go up
    public Command extend(){
        return Commands.runOnce(()-> io.setVoltage(6),this);
    }

    //go down
    public Command retract(){
        return Commands.runOnce(()-> io.setVoltage(-6), this);
    }

    //stop
    public Command stop(){
        return Commands.runOnce(()->io.setVoltage(0), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);        

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
