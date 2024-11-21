package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX elevatorMotor;
    private RelativeEncoder elevatorEncoder;

    public ElevatorIOTalonFX(int ID) {

        elevatorMotor = new TalonFX(ID);
        elevatorMotor.setSmartCurrentLimit(30);
        
        // elevatorMotor.setSmartCurrentLimit(30);
        // elevatorMotor.setIdleMode(IdleMode.kBrake);
        // elevatorMotor.setInverted(false);

        // elevatorEncoder = elevatorMotor.getEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.motorConnected = !(elevatorMotor.getFault(FaultID.kMotorFault) || elevatorMotor.getFault(FaultID.kCANRX) || elevatorMotor.getFault(FaultID.kCANRX));
        inputs.motorVolts = elevatorMotor.get()*RobotController.getBatteryVoltage();

        // inputs.motorConnected = !(elevatorMotor.getFault(FaultID.kMotorFault) || elevatorMotor.getFault(FaultID.kCANRX) || elevatorMotor.getFault(FaultID.kCANRX));
        // inputs.motorVolts = elevatorMotor.get()*RobotController.getBatteryVoltage();
        // inputs.motorCurrent = elevatorMotor.getOutputCurrent();
        // inputs.motorTemp = elevatorMotor.getMotorTemperature();
        // inputs.motorPosition = elevatorEncoder.getPosition();
    }
}
