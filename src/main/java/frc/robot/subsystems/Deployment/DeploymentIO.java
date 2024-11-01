package frc.robot.subsystems.Deployment;

import org.littletonrobotics.junction.AutoLog;

public interface DeploymentIO {
    @AutoLog
    public class DeploymentIOInputs {
        public boolean motorConnected = false;
        public double appliedVoltage = 0.0;
        public double appliedCurrent = 0.0;
        public double position = 0.0;
        public double motorTemp = 0.0;
    }

    public default void updateInputs(DeploymentIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default double getPosition() {
        return 0.0;
    }

    public default void stop() {}
}