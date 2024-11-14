package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
    
    private FlywheelSim intakeSim;

    private double volts;

    public IntakeIOSim() {
        intakeSim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
    }

    @Override
    public void setVoltage(double volts) {
        this.volts = volts;
        intakeSim.setInputVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeInput inputs) {
        inputs.motorConnection = true;
        inputs.motorVolts = this.volts;
        inputs.motorCurrent = intakeSim.getCurrentDrawAmps();
        inputs.motorTemp = 0.0;
        inputs.motorSpeed = intakeSim.getAngularVelocityRPM();
    }
    
}
