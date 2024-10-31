package frc.robot.subsystems.Deployment;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface DeploymentIO {
    @AutoLog
    public class DeploymentIOInputs {
        public boolean motorConnected = false;
        public double appliedVoltage = 0.0;
        public double appliedCurrent = 0.0;
        public double position = 0.0;
        public double motorTemp = 0.0;

        public Pose3d deploymentPose;
    }

    public default void updateInputs(DeploymentIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default double getPosition() {
        return 0.0;
    }
}