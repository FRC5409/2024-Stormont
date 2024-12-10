package frc.robot.subsystems.elevator;

import javax.management.DynamicMBean;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalon implements ElevatorIO {
    private TalonFX motor;

    public ElevatorIOTalon(int id) {
        motor = new TalonFX(id);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.getConfigurator().apply(config);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double setpoint, int slot) {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(slot);
        motor.setControl(m_request.withPosition(setpoint));
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.connected = motor.getFaultField().getValue() <= 0;
        inputs.voltage = motor.get() * RobotController.getBatteryVoltage();
        inputs.current = motor.getStatorCurrent().getValueAsDouble();
        inputs.temp = motor.getDeviceTemp().getValueAsDouble();
        inputs.speed = motor.getVelocity().getValueAsDouble();
    }

    public void setPID(double kP, double kI, double kD) {
        Slot0Configs pidConfig = new Slot0Configs();
        pidConfig.kP = kP;
        pidConfig.kI = kI;
        pidConfig.kD = kD;

        motor.getConfigurator().apply(pidConfig);
    }

}
