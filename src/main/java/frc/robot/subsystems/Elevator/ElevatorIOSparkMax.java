package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {

    private CANSparkMax elvatorMotor;
    private RelativeEncoder elevatorIncoder;

    public ElevatorIOSparkMax(int ID) {
        elvatorMotor = new CANSparkMax(ID, MotorType.kBrushless);
        
        elvatorMotor.restoreFactoryDefaults();
        elvatorMotor.setSmartCurrentLimit(30);
        elvatorMotor.setIdleMode(IdleMode.kBrake);
        elvatorMotor.setInverted(false);

        elvatorMotor.burnFlash();

        elevatorIncoder = elvatorMotor.getEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        elvatorMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs (ElevatorInputs inputs) {
        inputs.motorConnected = !(elvatorMotor.getFault(FaultID.kMotorFault) || elvatorMotor.getFault(FaultID.kCANRX) || elvatorMotor.getFault(FaultID.kCANRX));
        inputs.motorVolts = elvatorMotor.get()*RobotController.getBatteryVoltage();
        inputs.motorCurrent = elvatorMotor.getOutputCurrent();
        inputs.motorTemp = elvatorMotor.getMotorTemperature();
        inputs.motorSpeed = elevatorIncoder.getVelocity();
    }
}