package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim;
    private double volts = 0.0;

    public ElevatorIOSim() {
        sim = new ElevatorSim(LinearSystemId.createElevatorSystem(
                DCMotor.getNEO(1), 0, 0, 0),
                DCMotor.getNEO(1),
                0,
                2, // this is DEFINITELY not right 😭🙏
                true,
                0);
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
        this.volts = volts;
    }

    @Override
    public void setPosition(double setpoint, int slot) {
        sim.setState(setpoint, 0);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.connected = true;
        inputs.current = sim.getCurrentDrawAmps();
        inputs.speed = sim.getVelocityMetersPerSecond();
        inputs.temp = 0.0;
        inputs.voltage = this.volts;
    }

}
