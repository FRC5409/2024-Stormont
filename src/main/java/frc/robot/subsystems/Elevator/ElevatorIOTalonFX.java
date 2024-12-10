package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX motor;

    private final double kP = 0.5;
    private final double kI = 0;
    private final double kD = 0.05;

    public ElevatorIOTalonFX(int id) {
        motor = new TalonFX(id);
        TalonFXConfiguration config = new TalonFXConfiguration();
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        Slot0Configs pidConfig = new Slot0Configs();

        motor.setNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        voltageConfigs.PeakForwardVoltage = 3;
        voltageConfigs.PeakReverseVoltage = -3;

        pidConfig.kP = kP;
        pidConfig.kI = kI;
        pidConfig.kD = kD;

        motor.getConfigurator().apply(config);
        motor.getConfigurator().apply(voltageConfigs);
        motor.getConfigurator().apply(pidConfig);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setPosition(double setpoint, int slot) {
        motor.setControl(new PositionVoltage(setpoint)
            .withSlot(slot)
        );
    }

    @Override
    public double getP() {
        return kP;
    }
    
    @Override
    public double getI() {
        return kI;
    }

    @Override
    public double getD() {
        return kD;
    }

    @Override
    public String getMotorName() {
        return "TalonFX";
    }

    public double getPos() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.motorConnected = motor.getFaultField().getValue() <= 0;
        inputs.motorVoltageUsed = motor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.motorTemp = motor.getDeviceTemp().getValueAsDouble();
        inputs.motorSpeed = motor.getVelocity().getValueAsDouble();
    }
}