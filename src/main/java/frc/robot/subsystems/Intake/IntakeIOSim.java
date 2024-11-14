package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {

    private FlywheelSim intakeSim;
    private double volts;

    public IntakeIOSim() {
        intakeSim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
        //ElevatorSim sim = new ElevatorSim()
    }

    @Override
    public void setVoltage(double volts) {
        this.volts = volts;
        intakeSim.setInputVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.motorVolts = volts;
        inputs.motorConnected = true;
        inputs.motorCurrent = intakeSim.getCurrentDrawAmps();
        inputs.motorTemp = 0.0;
        inputs.motorSpeed = intakeSim.getAngularVelocityRadPerSec();
    }

}