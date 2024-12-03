package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax intakeMotor;
    private RelativeEncoder intakeIncoder;
    

    public IntakeIOSparkMax(int ID) {
        intakeMotor = new CANSparkMax(ID, MotorType.kBrushless);
        
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);

        intakeMotor.burnFlash();

        intakeIncoder = intakeMotor.getEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs (IntakeInputs inputs) {
        inputs.motorConnected = !(intakeMotor.getFault(FaultID.kMotorFault) || intakeMotor.getFault(FaultID.kCANRX) || intakeMotor.getFault(FaultID.kCANRX));
        inputs.motorVolts = intakeMotor.get()*RobotController.getBatteryVoltage();
        inputs.motorCurrent = intakeMotor.getOutputCurrent();
        inputs.motorTemp = intakeMotor.getMotorTemperature();
        inputs.motorSpeed = intakeIncoder.getVelocity();
    }
}