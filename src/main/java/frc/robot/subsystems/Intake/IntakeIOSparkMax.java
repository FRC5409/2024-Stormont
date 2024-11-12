package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.imageio.spi.ImageReaderWriterSpi;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private DigitalInput irSensor;
    
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
        inputs.motorConnected = !(motor.getFault(FaultID.kMotorFault) || motor.getFault(FaultID.kCANRX) || motor.getFault(FaultID.kCANTX));
        inputs.motorVoltage = motor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.motorSpeed = encoder.getVelocity();
        inputs.irTripped = !irSensor.get();
    }
}
