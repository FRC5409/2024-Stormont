package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog 
    public class ElevatorInput {
        public boolean elevatorConnection = false;
        public double elevatorVolts = 0.0;
        public double elevatorCurrent = 0.0;
        public double elevatorPosition = 0.0;
        public double elevatorTemp = 0.0;
    }

    public default void setVoltage(double volts) {}
    public default void setPosition(double value, int slot) {}
    public default void updateInputs(ElevatorInput inputs) {}
    public default String getName() {return "default";}
    public default void resetPosition (double position) {}
    public default void debugPID () {}
} 
