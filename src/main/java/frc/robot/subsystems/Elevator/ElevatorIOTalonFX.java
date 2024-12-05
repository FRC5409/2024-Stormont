package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX elevatorMotor;
    TalonFXConfigurator talonFXConfiguration;
    private CurrentLimitsConfigs currentConfig;
    private Slot0Configs slot0Configs = new Slot0Configs();
    private PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private final ShuffleboardTab sb_Talon;
    private final GenericEntry sb_kp;
    private final GenericEntry sb_ki;
    private final GenericEntry sb_kd;

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

        sb_Talon = Shuffleboard.getTab("Elevator -TalonFX");

        sb_Talon.addDouble("Position", () -> elevatorMotor.getPosition().getValueAsDouble());

        sb_kp = sb_Talon.add("kP", slot0Configs.kP).getEntry();
        sb_ki = sb_Talon.add("kI", slot0Configs.kI).getEntry();
        sb_kd = sb_Talon.add("kD", slot0Configs.kD).getEntry();
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
    public void resetPosition(double position) {
        elevatorMotor.setPosition(position);
    }

    @Override
    public void updateInputs(ElevatorInput inputs) {
        inputs.elevatorConnection = elevatorMotor.isAlive();
        inputs.elevatorVolts = elevatorMotor.get() * RobotController.getBatteryVoltage();
        inputs.elevatorCurrent = elevatorMotor.getSupplyCurrent().getValueAsDouble();
        inputs.elevatorTemp = elevatorMotor.getDeviceTemp().getValueAsDouble();
        inputs.elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
    }

    @Override
    public String getName() {
        return "Talong FX";
    }

    @Override
    public void debugPID() {
        SlotConfigs PIDConfig = new SlotConfigs();
        if (slot0Configs.kP != sb_kp.getDouble(0.0)) {
            slot0Configs.kP = sb_kp.getDouble(0.0);
            slot0Configs.kI = sb_ki.getDouble(0.0);
            slot0Configs.kD = sb_kd.getDouble(0.0);
        }
        elevatorMotor.getConfigurator().apply(PIDConfig);
    }

}
