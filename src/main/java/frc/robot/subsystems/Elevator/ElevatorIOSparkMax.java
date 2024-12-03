package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {
    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController sparkPid;

    public ElevatorIOSparkMax(int ID) {
        elevatorMotor = new CANSparkMax(ID, MotorType.kBrushless);
        
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(30);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(false);

        this.elevatorEncoder = elevatorMotor.getEncoder();

        elevatorMotor.burnFlash();
    }

    // Get subsystem
    @Override
    public void setVoltage(double volts){
        elevatorMotor.setVoltage(volts);
    }

    public void updateInputs(ElevatorInputs inputs){
        inputs.motorConnectedNEO = (!elevatorMotor.getFault(FaultID.kMotorFault) || elevatorMotor.getFault(FaultID.kCANRX) || elevatorMotor.getFault(FaultID.kCANTX));

        inputs.motorVoltsNEO = elevatorMotor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrentNEO = elevatorMotor.getOutputCurrent();
        inputs.motorTempNEO = elevatorMotor.getMotorTemperature();

        inputs.motorPositionNEO = elevatorEncoder.getPosition();
    }

    @Override
    public String getName(){
        return "Neo";
    }
    
}
