package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevatorSim;
    private double volts;
    private String name;

    public ElevatorIOSim(DCMotor motor, String name){
        this.name = name;
        elevatorSim = new ElevatorSim(motor, 9, 6, 0.0199, 1.066, 2.286, true, 1.066);
    }

    @Override
    public void setVoltage(double volts){
        this.volts = volts;
        elevatorSim.setInputVoltage(volts);
        elevatorSim.update(0.02);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs){
        inputs.motorConnectedNEO = true;
        inputs.motorVoltsNEO = this.volts;
        inputs.motorCurrentNEO = elevatorSim.getCurrentDrawAmps();
        inputs.motorTempNEO = 0.0;
        inputs.motorPositionNEO = elevatorSim.getPositionMeters();
        inputs.motorConnectedFalcon = true;
        inputs.motorVoltsFalcon = this.volts;
        inputs.motorCurrentFalcon = elevatorSim.getCurrentDrawAmps();
        inputs.motorTempFalcon = 0.0;
        inputs.motorPositionFalcon = elevatorSim.getPositionMeters();

    }

    @Override
    public String getName(){
        return name;
    }
}
