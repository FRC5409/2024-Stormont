package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeInputs {
        public boolean motorConnected = false;
        public double motorVoltage = 0.0;
        public double motorSpeed = 0.0;
        public double motorCurrent = 0.0;
        public double motorTemp = 0.0;
        // public boolean irTripped;
    }

    public default void setVoltage(double volts) {}

    public default void updateInputs(IntakeInputs inputs) {}
}
