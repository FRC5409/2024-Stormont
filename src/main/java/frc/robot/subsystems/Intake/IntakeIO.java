package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    // Motor Voltage 
    // Motor Speed
    // IR sensor (optional)
    // Motor Current
    // Motor Temp 
    // Motor Connection
    @AutoLog
    public class IntakeInput {
        public boolean motorConnection = false;
        public double motorVolts = 0.0;
        public double motorCurrent = 0.0;
        public double motorSpeed = 0.0;
        public double motorTemp = 0.0;
    }
    
    public default void setVoltage(double volts) {}

    public default void updateInputs(IntakeInput inputs) {}
}