package frc.robot.exception;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class ConfigMotorException extends RuntimeException {
    public ConfigMotorException(MotorController motor) {
        super(motor.toString() + " could not initialize");
    }
}
