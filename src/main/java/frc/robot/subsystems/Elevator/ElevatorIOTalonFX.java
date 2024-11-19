package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ElevatorIOTalonFX {
    private TalonFX elevatorMotor;
    private RelativeEncoder elevatorEncoder;

    public ElevatorIOTalonFX(int ID) {
        elevatorMotor = new TalonFX(ID, MotorType.kBrushless);
        
        elevatorMotor.restoreFactoryDefaults();
        // elevatorMotor.setSmartCurrentLimit(30);
        // elevatorMotor.setIdleMode(IdleMode.kBrake);
        // elevatorMotor.setInverted(false);

        // elevatorMotor.burnFlash();

        // elevatorEncoder = elevatorMotor.getEncoder();
    }
}
