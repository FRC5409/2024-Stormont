package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalonFX implements ElevatorIO{
    private TalonFX elevatorMotor;
    TalonFXConfigurator talonFXConfiguration;
    private CurrentLimitsConfigs currentConfig;
    private Slot0Configs slot0Configs = new Slot0Configs();
    private PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public ElevatorIOTalonFX(int ID) {
        elevatorMotor = new TalonFX(ID);
        talonFXConfiguration = elevatorMotor.getConfigurator();
        currentConfig = new CurrentLimitsConfigs();
        currentConfig.SupplyCurrentLimit = 30.0;
        currentConfig.SupplyCurrentLimitEnable = true;

        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setInverted(true); 

        talonFXConfiguration.apply(currentConfig);

        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        elevatorMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void setPosition(double value) {
        elevatorMotor.setControl(m_request.withPosition(value));
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
