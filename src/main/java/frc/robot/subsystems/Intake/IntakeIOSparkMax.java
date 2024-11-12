package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax intakeMotor; //mot_intake, m_motor, motorIntake, motor (Acceptable names for var)
    private RelativeEncoder intakEncoder;//s_encoder, m_encodor, encoderIntake (Acceptable names for var)

    public IntakeIOSparkMax(int ID){
        intakeMotor = new CANSparkMax(ID, MotorType.kBrushless);
        
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);

        intakeMotor.burnFlash();//saves the data into the actual sparkMax

        intakEncoder = intakeMotor.getEncoder();
    }
    @Override
    public void setVoltage(double volts){
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeInputs inputs){
        inputs.motorConnected = true;

        inputs.motorVolts = intakeMotor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = intakeMotor.getOutputCurrent();
        inputs.motorTemp = intakeMotor.getMotorTemperature();

        inputs.motorSpeed = intakEncoder.getVelocity();
    }
}
