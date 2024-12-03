package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalonFx implements ElevatorIO{
    private TalonFX elevatorMotor;
    private TalonFXConfigurator talonFXConfiguration;
    private CurrentLimitsConfigs currentConfig;
    Slot0Configs slot0Configs = new Slot0Configs();
    // create a position closed-loop request, voltage output, slot 0 configs
    private PositionVoltage m_request = new PositionVoltage(0).withSlot(0);



    public ElevatorIOTalonFx(int ID) {
        elevatorMotor = new TalonFX(ID);
        talonFXConfiguration = elevatorMotor.getConfigurator();
        currentConfig = new CurrentLimitsConfigs();
        currentConfig.SupplyCurrentLimit = 30.0;
        currentConfig.SupplyCurrentLimitEnable = true;

        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setInverted(true); 

        talonFXConfiguration.apply(currentConfig);

        //PID
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
    public void setPosition(double value){
        // set position to value rotations
        elevatorMotor.setControl(m_request.withPosition(value));

    }

    @Override
    public void resetPosition(double resetValue){
        elevatorMotor.setPosition(resetValue);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.motorConnectedFalcon = elevatorMotor.isAlive();

        inputs.motorVoltsFalcon = elevatorMotor.get() *RobotController.getBatteryVoltage();
        inputs.motorCurrentFalcon = elevatorMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorTempFalcon = elevatorMotor.getDeviceTemp().getValueAsDouble();

        inputs.motorPositionFalcon = elevatorMotor.getPosition().getValueAsDouble();
    }

    

    @Override 
    public String getName() {
        return "Talong FX";
    }
    
}
