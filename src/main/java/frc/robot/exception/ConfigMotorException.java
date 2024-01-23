package frc.robot.exception;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class ConfigMotorException extends RuntimeException {
    public ConfigMotorException(MotorController motor) {
        super(motor.toString() + " could not initialize");
    }
}
