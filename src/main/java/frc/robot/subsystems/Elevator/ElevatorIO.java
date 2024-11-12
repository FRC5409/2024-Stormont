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
        public boolean motorConnected = false;
        public double motorVolts = 0.0;
        public double motorCurrent = 0.0;
        public double motorPosition = 0.0;
        public double motorTemp = 0.0;
    }

    public default void setVoltage(double volts){}
    public default void updateInputs(ElevatorInputs inputs){}
}
