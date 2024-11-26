package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {

    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;

    public ElevatorIOSparkMax(int elevatorIOSparkMax) {
        elevatorMotor = new CANSparkMax(elevatorIOSparkMax, MotorType.kBrushless);
        
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(30);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(false);

        elevatorMotor.burnFlash();

        elevatorEncoder = elevatorMotor.getEncoder();
    }

    public ElevatorIOSparkMax() {
        //TODO Auto-generated constructor stub
    }

    public ElevatorIOSparkMax(ElevatorIOSparkMax elevatorIOSparkMax) {
        //TODO Auto-generated constructor stub
    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs (ElevatorInputs inputs) {
        inputs.motorConnected = !(elevatorMotor.getFault(FaultID.kMotorFault) || elevatorMotor.getFault(FaultID.kCANRX) || elevatorMotor.getFault(FaultID.kCANRX));
        inputs.motorVolts = elevatorMotor.get()*RobotController.getBatteryVoltage();
        inputs.motorCurrent = elevatorMotor.getOutputCurrent();
        inputs.motorTemp = elevatorMotor.getMotorTemperature();
        inputs.motorPosition = elevatorEncoder.getPosition();
    }
}