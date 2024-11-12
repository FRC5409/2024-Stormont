package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    //motor voltage
    //motor speed
    //IR sensor (optional)
    //motor current
    //motor temp
    //motor connection
    
    @AutoLog
    public class IntakeInputs {
        public boolean moterConnected = false;
        public double motorVoltage = 0.0;
        public double motorCurrent = 0.0;
        public double motorSpeed = 0.0;
        public double motorTemp = 0.0;
    }

    public default void setVoltage(double volts) {}
    
    public default void updateInputs(IntakeInputs inputs) {}
}
