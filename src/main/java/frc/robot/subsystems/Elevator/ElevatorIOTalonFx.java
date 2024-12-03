package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalonFX implements ElevatorIO{
    private TalonFX elevatorMotor;
    TalonFXConfigurator talonFXConfiguration;
    private CurrentLimitsConfigs currentConfig;

    public ElevatorIOTalonFX(int ID) {
        elevatorMotor = new TalonFX(ID);
        talonFXConfiguration = elevatorMotor.getConfigurator();
        currentConfig = new CurrentLimitsConfigs();
        currentConfig.SupplyCurrentLimit = 30.0;
        currentConfig.SupplyCurrentLimitEnable = true;

        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setInverted(true); 

        talonFXConfiguration.apply(currentConfig);
    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ElevatorInput inputs) {
        inputs.elevatorConnection = elevatorMotor.isAlive();

        inputs.elevatorVolts = elevatorMotor.get() *RobotController.getBatteryVoltage();
        inputs.elevatorCurrent = elevatorMotor.getSupplyCurrent().getValueAsDouble();
        inputs.elevatorTemp = elevatorMotor.getDeviceTemp().getValueAsDouble();

        inputs.elevatorPositionTalon = elevatorMotor.getPosition().getValueAsDouble();
    }

    @Override 
    public String getName() {
        return "Talong FX";
    }
    
}
