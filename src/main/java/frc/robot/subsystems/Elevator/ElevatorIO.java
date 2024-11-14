package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import edu.wpi.first.units.Voltage;

public interface ElevatorIO {

    @AutoLog
    public class ElevatorInputs {
        public boolean motorConnected = false;
        public double motorCurrent = 0.0;
        public double motorVolts = 0.0;
        public double motorSpeed = 0.0;
        public double motorTemp = 0.0;
    }

    public default void setVoltage(double volts) {}

    public default void updateInputs(ElevatorInputs inputs) {}

}
