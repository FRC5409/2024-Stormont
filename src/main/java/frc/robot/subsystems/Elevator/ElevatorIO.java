package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorInputs {
        public boolean motorConnected = false;
        public double motorCurrent = 0.0;
        public double motorVolts = 0.0;
        public double motorPosition = 0.0;
        public double motorSpeed = 0.0;
        public double motorTemp = 0.0;
    }

    public default void setVoltage(double volts) {}

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setPosition(double position) {}

    // Up ten rotations
    public default void setPositionUp(double position) {}

    // Down ten rotations
    public default void setPositionDown(double position) {}
}