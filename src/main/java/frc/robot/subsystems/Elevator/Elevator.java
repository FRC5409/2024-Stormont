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
        Shuffleboard.getTab("Elevator")
            .addDouble("NeoMotorPosition"+ io.getName(), () -> inputs.motorPositionNEO);
        
    }

    // Get subsystem

    //go up
    public Command extend(double value, int slot){
        return Commands.runOnce(()-> io.setPosition(value, slot),this);
    }

    //go down
    public Command retract(double value, int slot){
        return Commands.runOnce(()-> io.setPosition(value, slot), this);
    }

    //stop
    public Command stop(){
        return Commands.runOnce(()->io.setVoltage(0), this);
    }

    // Go up manually
    public Command m_extend(){
        return Commands.runOnce(()->io.setVoltage(6), this);
    }

    // Go down manually
    public Command m_retract(){
        return Commands.runOnce(()->io.setVoltage(-6), this);
    }

    //reset it to 0
    public Command reset(){
        return Commands.runOnce(()->io.resetPosition(0), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);        
        io.debugPID();

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
