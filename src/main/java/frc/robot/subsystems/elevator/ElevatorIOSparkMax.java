package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController controller;

    public ElevatorIOSparkMax(int id) {
        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
        motor.burnFlash();

        encoder = motor.getEncoder();

        controller = motor.getPIDController();

        // TOOD: PID tuning
        controller.setP(0, 0);
		controller.setI(0, 0);
		controller.setD(0, 0);
		controller.setOutputRange(-1.0, 1.0, 0);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double setpoint, int slot) {
		controller.setReference(setpoint, ControlType.kPosition, slot);
	}

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.connected = !(motor.getFault(FaultID.kCANRX) || motor.getFault(FaultID.kCANTX)
                || motor.getFault(FaultID.kMotorFault));
        inputs.voltage = motor.get() * RobotController.getBatteryVoltage();
        inputs.current = motor.getOutputCurrent();
        inputs.temp = motor.getMotorTemperature();
        inputs.speed = encoder.getVelocity();
    }
}
