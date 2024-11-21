package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.kClimber.kSpark;

public class ClimberIOSparkMax implements ClimberIO {

    private final CANSparkMax mot;
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;
    
    public ClimberIOSparkMax(int ID) {
        mot = new CANSparkMax(ID, MotorType.kBrushless);

        mot.restoreFactoryDefaults();
        mot.setSmartCurrentLimit(kSpark.CURRENT_LIMIT);
        mot.setInverted(kSpark.INVERTED);
        mot.setIdleMode(IdleMode.kBrake);

        encoder = mot.getEncoder();

        encoder.setPositionConversionFactor(kSpark.CONVERSION_FACTOR);
        encoder.setVelocityConversionFactor(kSpark.CONVERSION_FACTOR / 50.0);

        controller = mot.getPIDController();

        controller.setP(kSpark.kP);
        controller.setI(kSpark.kI);
        controller.setD(kSpark.kD);
        controller.setFF(kSpark.kFF);

        mot.burnFlash();
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.motorConnected = true;
        inputs.motorVoltage = mot.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = mot.getOutputCurrent();
        inputs.climberPosition = encoder.getPosition();
        inputs.motorTemp = mot.getMotorTemperature();
    }

    @Override
    public void setPosition(double position) {
        controller.setReference(position, ControlType.kPosition);
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
    public String getIOName() {
        return "SparkMax";
    }
}
