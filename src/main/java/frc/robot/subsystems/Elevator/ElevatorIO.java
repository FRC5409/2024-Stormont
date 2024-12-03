package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    // Motor Connected
    // Motor Voltage
    // Motor Speed
    // IR Sensor(optional)
    // Motor Current
    // Motor Temp
    @AutoLog
    public class ElevatorInputs{
        public boolean motorConnectedNEO = false;
        public double motorVoltsNEO = 0.0;
        public double motorCurrentNEO = 0.0;
        public double motorPositionNEO = 0.0;
        public double motorTempNEO = 0.0;
        public boolean motorConnectedFalcon = false;
        public double motorVoltsFalcon = 0.0;
        public double motorCurrentFalcon = 0.0;
        public double motorPositionFalcon = 0.0;
        public double motorTempFalcon = 0.0;
    }

    public default void setVoltage(double volts){}
    public default void setPosition(double value){}
    public default void resetPosition(double resetValue){}
    public default void updateInputs(ElevatorInputs inputs){}
    public default String getName( ){return "name";}
}
