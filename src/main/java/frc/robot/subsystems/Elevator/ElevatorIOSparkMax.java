package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {
    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkPIDController pid;
    private double kP = 0.0, kI = 0.0, kD = 0.0;

    

    public ElevatorIOSparkMax(int ID) {
        elevatorMotor = new CANSparkMax(ID, MotorType.kBrushless);
        
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(30);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(false);

        this.elevatorEncoder = elevatorMotor.getEncoder();

        //Change Values Through Shuffleboard


        pid = elevatorMotor.getPIDController();
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);

        elevatorMotor.burnFlash();


    }

    // Get subsystem
    @Override
    public void setVoltage(double volts){
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void setPosition(double value){
    pid.setReference(value,ControlType.kPosition);
    }

    public void updateInputs(ElevatorInputs inputs){
        inputs.motorConnectedNEO = !(elevatorMotor.getFault(FaultID.kMotorFault) || elevatorMotor.getFault(FaultID.kCANRX) || elevatorMotor.getFault(FaultID.kCANTX));

        inputs.motorVoltsNEO = elevatorMotor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrentNEO = elevatorMotor.getOutputCurrent();
        inputs.motorTempNEO = elevatorMotor.getMotorTemperature();

        inputs.motorPositionNEO = elevatorEncoder.getPosition();
    }

    @Override
    public void resetPosition(double resetValue){
        if(0<=resetValue || resetValue<=1.1){
            elevatorEncoder.setPosition(resetValue);
        } else if(resetValue<0){
            resetValue = 0;
            elevatorEncoder.setPosition(resetValue);
        } else if(resetValue>=1.2){
            resetValue = 1.1;
            elevatorEncoder.setPosition(resetValue);
        }
    }

    @Override
    public String getName(){
        return "Neo";
    }
    
}
