package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOPhysical implements ElevatorIO {
    private CANSparkMax motor;
    private RelativeEncoder encoder;

    private PIDController pid;

    public ElevatorIOPhysical() {
        motor = new CANSparkMax(0, null)
    }
}
