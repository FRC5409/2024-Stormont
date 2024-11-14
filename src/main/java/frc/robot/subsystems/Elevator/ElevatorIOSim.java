package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIO.IntakeInputs;

public class ElevatorIOSim implements ElevatorIO {
    private FlywheelSim intakeSim;
    private double volts;

    public ElevatorIOSim() {
        intakeSim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
        //ElevatorSim sim = new ElevatorSim()
    }

    @Override
    public void setVoltage(double volts) {
        this.volts = volts;
        intakeSim.setInputVoltage(volts);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.motorVolts = volts;
        inputs.motorConnected = true;
        inputs.motorCurrent = intakeSim.getCurrentDrawAmps();
        inputs.motorTemp = 0.0;
        inputs.motorSpeed = intakeSim.getAngularVelocityRadPerSec();
}
