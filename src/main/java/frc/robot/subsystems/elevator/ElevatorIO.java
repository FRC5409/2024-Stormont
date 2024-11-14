package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorInputs {
        public boolean connected = false;
        public double voltage = 0.0;
        public double speed = 0.0;
        public double current = 0.0;
        public double temp = 0.0;
    }

    public default void setVoltage(double volts) {}

    public default void updateInputs(ElevatorInputs inputs) {} 
}