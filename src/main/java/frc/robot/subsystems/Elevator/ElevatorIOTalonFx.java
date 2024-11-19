package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalonFx implements ElevatorIO {
    private TalonFX elevatorMotor;
    private RelativeEncoder elevatorEncoder;

    public ElevatorIOTalonFx(int ID) {
        elevatorMotor = new TalonFX(ID);
        
        // elevatorMotor.restoreFactoryDefaults();
        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs currentOutputConfig = new MotorOutputConfigs();
        currentConfig.SupplyCurrentLimit = 30.0;
        currentConfig.SupplyCurrentLimitEnable = true;
        
        currentOutputConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorMotor.setInverted(false);

        // elevatorMotor.burnFlash();
    }

    // Get subsystem
    @Override
    public void setVoltage(double volts){
        elevatorMotor.setVoltage(volts);
    }

    public void updateInputs(ElevatorInputs inputs){
        inputs.motorConnected = (!elevatorMotor.getFault(FaultID.kMotorFault) || elevatorMotor.getFault(FaultID.kCANRX) || elevatorMotor.getFault(FaultID.kCANTX));

        inputs.motorVolts = elevatorMotor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = elevatorMotor.getOutputCurrent();
        inputs.motorTemp = elevatorMotor.getMotorTemperature();

        inputs.motorPosition = elevatorEncoder.getPosition();
    }
}
