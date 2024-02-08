package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    private static Indexer instance = null;

    private final CANSparkMax motor;

    private Indexer() {
        motor = initMotor();
    }

    private CANSparkMax initMotor() {
        CANSparkMax motor = new CANSparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setOpenLoopRampRate(IndexerConstants.RAMP_RATE);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(IndexerConstants.CURRENT_LIMIT);

        motor.burnFlash();
        return motor;
    }

    // Get subsystem
    public static Indexer getInstance() { if (instance == null) instance = new Indexer(); return instance; }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
}
