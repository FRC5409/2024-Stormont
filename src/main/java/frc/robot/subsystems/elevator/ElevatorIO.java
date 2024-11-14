package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorInputs {
        public boolean motorConnected = false;
        public double motorVoltage = 0.0;
        public double motorSpeed = 0.0;
        public double motorCurrent = 0.0;
        public double motorTemp = 0.0;
    }

    public void setVoltage(double volts);

    public void updateInputs(ElevatorInputs inputs);
}