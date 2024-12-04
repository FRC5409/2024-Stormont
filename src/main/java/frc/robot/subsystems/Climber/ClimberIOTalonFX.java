package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.kClimber.kTalon;

public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX mot;
    private final PositionVoltage PIDControl;

    private final StatusSignal<Double> motorVoltage;
    private final StatusSignal<Double> motorCurrent;
    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> motorTemp;
    
    public ClimberIOTalonFX(int ID) {
        mot = new TalonFX(ID);

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        currentConfig.SupplyCurrentLimit = kTalon.CURRENT_LIMIT;
        currentConfig.SupplyCurrentLimitEnable = true;

        FeedbackConfigs encoderConfig = new FeedbackConfigs();
        encoderConfig.SensorToMechanismRatio = kTalon.CONVERSION_FACTOR;

        SlotConfigs PIDConfig = new SlotConfigs();
        PIDConfig.kP = kTalon.kP;
        PIDConfig.kI = kTalon.kI;
        PIDConfig.kD = kTalon.kD;

        mot.getConfigurator().apply(currentConfig);
        mot.getConfigurator().apply(encoderConfig);
        mot.getConfigurator().apply(PIDConfig);
        
        mot.setInverted(false);
        mot.setNeutralMode(NeutralModeValue.Brake);

        // Set status singals (100hz recommended by phoenix)
        motorVoltage = mot.getMotorVoltage();
        motorVoltage.setUpdateFrequency(100);

        motorCurrent = mot.getSupplyCurrent();
        motorCurrent.setUpdateFrequency(100);

        motorPosition = mot.getPosition();
        motorPosition.setUpdateFrequency(100);

        motorTemp = mot.getDeviceTemp();
        motorTemp.setUpdateFrequency(100);

        PIDControl = new PositionVoltage(0)
            .withEnableFOC(true)
            .withUpdateFreqHz(50);
        
        // Optimize CAN bus. Removes all of the other status signals from updating, saving network data
        mot.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.motorConnected = mot.isAlive();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorCurrent = motorCurrent.getValueAsDouble();
        inputs.climberPosition = motorPosition.getValueAsDouble();
        inputs.motorTemp = motorTemp.getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
        mot.setControl(PIDControl.withPosition(position));
    }

    @Override
    public void stop() {
        mot.stopMotor();
    }

    @Override
    public void setVoltage(double voltage) {
        mot.setVoltage(voltage);
    }

    @Override
    public void zero() {
        mot.setPosition(0.0);
    }

    @Override
    public String getIOName() {
        return "TalonFX";
    }
}
