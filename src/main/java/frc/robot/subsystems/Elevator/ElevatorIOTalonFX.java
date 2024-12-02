package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX elevatorMotor;

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

        //PID
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        elevatorMotor.getConfigurator().apply(slot0Configs);
    }

    public ElevatorIOTalonFX(){}

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override 
    public void up10(){
        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        // set position to 10 rotations
        elevatorMotor.setControl(m_request.withPosition(10));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.motorConnected = elevatorMotor.getFaultField().getValue()==0;
        inputs.motorVolts = elevatorMotor.getMotorVoltage().getValue();
        inputs.motorCurrent = elevatorMotor.getStatorCurrent().getValue();
        inputs.motorPosition = elevatorMotor.getPosition().getValue();
        inputs.motorTemp = elevatorMotor.getDeviceTemp().getValue();
        inputs.motorSpeed = elevatorMotor.getVelocity().getValue();
    }
}
