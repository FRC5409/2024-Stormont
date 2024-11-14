package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOSim implements IntakeIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput irSensor;

    public IntakeIOSim(int motorID) {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(30);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
        motor.burnFlash();

        encoder = motor.getEncoder();

        irSensor = new DigitalInput(0);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.motorConnected = !(motor.getFault(FaultID.kMotorFault) || motor.getFault(FaultID.kCANRX) || motor.getFault(FaultID.kCANTX));
        inputs.motorVoltageUsed = motor.get() * RobotController.getBatteryVoltage();
        inputs.motorSpeed = motor.get();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.irTripped = irSensor.get();
    }
}