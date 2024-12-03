package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOSparkMax implements ElevatorIO{

    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkPIDController pid;

    public ElevatorIOSparkMax(int ID) {
        elevatorMotor = new CANSparkMax(ID, MotorType.kBrushless);

        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(30);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(false);
        elevatorMotor.burnFlash();
        elevatorEncoder = elevatorMotor.getEncoder();

        pid = elevatorMotor.getPIDController();
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
    }

    @Override
    public void setPosition(double value) {
        pid.setReference(value, ControlType.kPosition);
    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ElevatorInput inputs) {
        inputs.elevatorConnection = !(elevatorMotor.getFault(FaultID.kMotorFault)|| elevatorMotor.getFault(FaultID.kCANRX)|| elevatorMotor.getFault(FaultID.kCANTX));

        inputs.elevatorVolts = elevatorMotor.get() * RobotController.getBatteryVoltage();
        inputs.elevatorCurrent = elevatorMotor.getOutputCurrent();
        inputs.elevatorTemp = elevatorMotor.getMotorTemperature();

        inputs.elevatorPositionNEO = elevatorEncoder.getPosition();
     }

     @Override
     public String getName() {
        return "NEO";
     }
    
}
