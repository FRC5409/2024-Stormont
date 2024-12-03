package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog 
    public class ElevatorInput {
        public boolean elevatorConnection = false;
        public double elevatorVolts = 0.0;
        public double elevatorCurrent = 0.0;
        public double elevatorPositionTalon = 0.0;
        public double elevatorPositionNEO = 0.0;
        public double elevatorTemp = 0.0;
    }

    public default void setVoltage(double volts) {}
    public default void setPosition(double value) {}
    public default void updateInputs(ElevatorInput inputs) {}
    public default String getName(){return "";}
    
} 
