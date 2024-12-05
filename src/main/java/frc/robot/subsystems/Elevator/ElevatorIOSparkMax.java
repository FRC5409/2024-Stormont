package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.kPIDConstants;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {

    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkPIDController pidController;

    public ElevatorIOSparkMax(int elevatorIOSparkMax) {
        elevatorMotor = new CANSparkMax(elevatorIOSparkMax, MotorType.kBrushless);

        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(30);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(false);
        
        pidController = elevatorMotor.getPIDController();
        pidController.setP(kPIDConstants.kPSpark);
        pidController.setI(kPIDConstants.kISpark);
        pidController.setD(kPIDConstants.kDSpark);   
        
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
    public void upTen() {
        pidController.setReference(10, CANSparkBase.ControlType.kPosition);
    }

    @Override
    public void resetPosition() {
        elevatorMotor.setVoltage(-6);
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