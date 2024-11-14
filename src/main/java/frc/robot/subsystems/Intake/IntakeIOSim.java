package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
    private final FlywheelSim motor;
    private double volts;

    public IntakeIOSim(int motorID) {
        motor = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setInputVoltage(volts);
        this.volts = volts;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.motorConnected = true;
        inputs.motorVoltageUsed = this.volts;
        inputs.motorSpeed = motor.getAngularVelocityRadPerSec();
        inputs.motorCurrent = motor.getCurrentDrawAmps();
        inputs.motorTemp = 0.0;
    }
}