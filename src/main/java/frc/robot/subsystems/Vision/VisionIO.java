package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs {
        public boolean isConnected = false;
        public Pose2d pose = new Pose2d();

        public double fps = 0.0;
        public double cpuTemp = 0.0;
        public double ramUsage = 0.0;
        public double temp = 0.0;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default String getName() {
        return "noname";
    }

    public default PoseEstimate getEstimatedPose() {
        return null;
    }
}