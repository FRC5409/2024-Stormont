package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    
    public IntakeIOSparkMax(int id) {
        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(30);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
        motor.burnFlash();

        encoder = motor.getEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.motorVoltage = motor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.motorSpeed = encoder.getVelocity();
        inputs.motorConnected = true;
    }
}
