package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalonFx implements ElevatorIO {
    private TalonFX elevatorMotor;

    public ElevatorIOTalonFx(int ID) {
        elevatorMotor = new TalonFX(ID);
        
        // elevatorMotor.restoreFactoryDefaults();
        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        TalonFXConfigurator talonFXConfigurator = elevatorMotor.getConfigurator();
        currentConfig.SupplyCurrentLimit = 30.0;
        currentConfig.SupplyCurrentLimitEnable = true;

        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setInverted(false);

        talonFXConfigurator.apply(currentConfig);

        // elevatorMotor.burnFlash();
    }

    // Get subsystem
    @Override
    public void setVoltage(double volts){
        elevatorMotor.setVoltage(volts);
    }

    public void updateInputs(ElevatorInputs inputs){
        inputs.motorConnected = elevatorMotor.isAlive();
        inputs.motorVolts = elevatorMotor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = elevatorMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorTemp = elevatorMotor.getDeviceTemp().getValueAsDouble();

        inputs.motorPositionTalon = elevatorMotor.getPosition().getValueAsDouble();
    }

    @Override
    public String getName(){
        return "Talon";
    }
}
