package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public class ClimberInputs {
        public boolean motorConnected = false;
        public double motorVoltage = 0.0;
        public double motorCurrent = 0.0;
        public double climberPosition = 0.0;
    }

    public default void updateInputs(ClimberInputs inputs) {}

    public default void setPosition(double position) {}

    public default void stop() {}

    public default String getIOName() {
        return "default";
    }
}
