package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalon implements ElevatorIO {
    private TalonFX motor;
    private RelativeEncoder encoder;

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
    public void updateInputs(ElevatorInputs inputs) {
        inputs.connected = motor.getFaultField().getValue() <= 0;
        inputs.voltage = motor.get() * RobotController.getBatteryVoltage();
        inputs.current = motor.getStatorCurrent().getValueAsDouble();
        inputs.temp = motor.getDeviceTemp().getValueAsDouble();
        inputs.speed = encoder.getVelocity();
    }
}
