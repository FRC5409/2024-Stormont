package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevatorSim;
    private double volts;

    public ElevatorIOSim(DCMotor motor){
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
        inputs.motorConnected = true;
        inputs.motorVolts = this.volts;
        inputs.motorCurrent = elevatorSim.getCurrentDrawAmps();
        inputs.motorTemp = 0.0;
        inputs.motorPosition = elevatorSim.getPositionMeters();
    }
}
