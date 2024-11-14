package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {
    private CANSparkMax motor;
    private RelativeEncoder encoder;

    public ElevatorIOSparkMax(int id) {
        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);
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
    public void updateInputs(ElevatorInputs inputs) {
        inputs.motorConnected = !(motor.getFault(FaultID.kCANRX) || motor.getFault(FaultID.kCANTX)
                || motor.getFault(FaultID.kMotorFault));
        inputs.motorVoltage = motor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.motorSpeed = encoder.getVelocity();
    }
}
