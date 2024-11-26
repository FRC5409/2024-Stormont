package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalon implements ElevatorIO {
    private TalonFX motor;

    public ElevatorIOTalon(int id) {
        motor = new TalonFX(id);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor.setNeutralMode(NeutralModeValue.Brake);

        // TOOD: PID tuning
        Slot0Configs pidConfig = new Slot0Configs();
        pidConfig.kP = 0;
        pidConfig.kI = 0;
        pidConfig.kD = 0;

        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(pidConfig);
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
    public void updateInputs(ElevatorInputs inputs) {
        inputs.connected = motor.getFaultField().getValue() <= 0;
        inputs.voltage = motor.get() * RobotController.getBatteryVoltage();
        inputs.current = motor.getStatorCurrent().getValueAsDouble();
        inputs.temp = motor.getDeviceTemp().getValueAsDouble();
        inputs.speed = motor.getVelocity().getValueAsDouble();
    }
}
