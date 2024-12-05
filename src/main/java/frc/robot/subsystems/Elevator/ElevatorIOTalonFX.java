package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX elevatorMotor;
    private final PositionVoltage positionVoltage;

    public ElevatorIOTalonFX(int elevatorIOTalonFX) {
        elevatorMotor = new TalonFX(elevatorIOTalonFX);
        
        var configurator = elevatorMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();
        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 30;
        limitConfigs.StatorCurrentLimitEnable = true;
        configurator.apply(limitConfigs);

        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
      
        elevatorMotor.setInverted(false);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        elevatorMotor.getConfigurator().apply(slot0Configs);
        positionVoltage = new PositionVoltage(0).withSlot(0);

    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override 
    public void upTen() {
        elevatorMotor.setControl(positionVoltage.withPosition(10));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.motorConnected = elevatorMotor.getFaultField().getValue() == 0;
        inputs.motorVolts = elevatorMotor.getMotorVoltage().getValue();
        inputs.motorCurrent = elevatorMotor.getStatorCurrent().getValue();
        inputs.motorPosition = elevatorMotor.getPosition().getValue();
        inputs.motorTemp = elevatorMotor.getDeviceTemp().getValue();
        inputs.motorSpeed = elevatorMotor.getVelocity().getValue();
    }
}
