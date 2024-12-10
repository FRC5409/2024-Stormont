package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOSparkMax implements ElevatorIO {

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController controller;

    public ElevatorIOSparkMax(int id) {
        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);
        motor.burnFlash();

        encoder = motor.getEncoder();

        controller = motor.getPIDController();

        controller.setP(0.1, 0);
		controller.setI(0, 0);
		controller.setD(0.015, 0);
		controller.setOutputRange(-0.3, 0.3, 0);
    }

    @Override
    public double getP() {
        return controller.getP();
    }
    
    @Override
    public double getI() {
        return controller.getI();
    }

    @Override
    public double getD() {
        return controller.getD();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public String getMotorName() {
        return "Spark";
    }

    @Override
    public void setPosition(double setpoint, int slot) {
		controller.setReference(setpoint, ControlType.kPosition, slot);
	}

    @Override
    public double getPos() {
        return encoder.getPosition();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.motorConnected = !(motor.getFault(FaultID.kCANRX)
            || motor.getFault(FaultID.kCANTX)
            || motor.getFault(FaultID.kMotorFault));
        inputs.motorVoltageUsed = motor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.motorSpeed = encoder.getVelocity();
    }
}