package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO{

    private ElevatorSim elevatorSim;

    private double volts;

    public ElevatorIOSim() {
        elevatorSim = new ElevatorSim(DCMotor.getNEO(1), 1, 6, 0.0199, 1.066, 2.286, true, 1.066);
    }

    @Override
    public void setVoltage(double volts) {
        this.volts = volts;
        elevatorSim.setInputVoltage(volts);
        elevatorSim.update(0.02);
    }

    @Override
    public void updateInputs(ElevatorInput inputs) {
        inputs.elevatorConnection = true;
        inputs.elevatorVolts = this.volts;
        inputs.elevatorCurrent = elevatorSim.getCurrentDrawAmps();
        inputs.elevatorTemp = 0.0;
        inputs.elevatorPosition = elevatorSim.getPositionMeters();
    }
    
}
