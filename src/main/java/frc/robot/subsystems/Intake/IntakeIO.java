package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    // Motor voltage
    // Motor speed
    // IR Sensor (optional)
    // Motor current
    // Motor temp
    // Motor connected
    @AutoLog
    public class IntakeInputs {
        public boolean motorConnected = false;
        public double motorVolts = 0.0;
        public double motorCurrent = 0.0;
        public double motorSpeed = 0.0;
        public double motorTemp = 0.0;
    }

    public default void setVoltage(double volts) {}

    public default void updateInputs(IntakeInputs inputs) {}
}
