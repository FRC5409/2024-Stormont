package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
    private FlywheelSim motor;
    private double volts;

    public IntakeIOSim() {
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
        inputs.motorCurrent = motor.getCurrentDrawAmps();
        inputs.motorSpeed = motor.getAngularVelocityRadPerSec();
        inputs.motorTemp = 0.0;
        inputs.motorVoltage = this.volts;
    }
}
