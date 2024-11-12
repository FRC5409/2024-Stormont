package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;

public class IntakeIOSparkMax implements IntakeIO {
    
    //Motor
    private CANSparkMax m_motor;

    //Encoder
    private RelativeEncoder s_encoder;

    public IntakeIOSparkMax(int ID) {

        //Initialize motor
        m_motor = new CANSparkMax(ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(30);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(false);

        m_motor.burnFlash();

        //Initialize encoder
        s_encoder = m_motor.getEncoder();

    }

    /**
     * Sets voltage for manual extension
     * 
     * @param volts
     */

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    /**
     * Sets voltage for manual extension
     * 
     * @param inputs
     */

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.moterConnected = !(m_motor.getFault(FaultID.kMotorFault) || m_motor.getFault(FaultID.kCANRX) || m_motor.getFault(FaultID.kCANTX));

        inputs.motorVoltage = m_motor.get() * RobotController.getBatteryVoltage();
        inputs.motorCurrent = m_motor.getOutputCurrent();
        inputs.motorTemp = m_motor.getMotorTemperature();

        inputs.motorSpeed = s_encoder.getVelocity();

    }
}
