package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorInputs {
        public boolean motorConnected = false;
        public double motorVoltageUsed = 0.0;
        public double motorSpeed = 0.0;
        public double motorCurrent = 0.0;
        public double motorTemp = 0.0;
        public boolean irTripped = false;
    }

    public default void setVoltage(double volts) {}

    public default void setSpeed(double speed) {}

    public default void setPosition(double setpoint, int slot) {}

    public default double getP() {return 0.00;}

    public default double getI() {return 0.00;}

    public default double getD() {return 0.00;}

    public default String getMotorName() {return "";}
    
    public default double getPos() {return 0.00;}

    public default void updateInputs(ElevatorInputs inputs) {}
}