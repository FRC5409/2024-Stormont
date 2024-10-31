package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;

import java.util.Arrays;

public class VisionIOLimelight implements VisionIO {
    public final String name;

    private double[] latency = new double[4];

    public VisionIOLimelight(Transform3d offset) {
        this(null, offset);
    }

    public VisionIOLimelight(String name, Transform3d offset) {
        this(name, offset, 5800);
    }

    public VisionIOLimelight(String name, Transform3d offset, int port) {
        String limelightPath;
        // TODO: Might need to move this to robotinit (for some reason???)
        if (name == null || name.equals(""))
            limelightPath = "limelight.local";
        else
            limelightPath = "limelight-" + name + ".local";
            
        for (int i = port; i < port + 10; port++) {
            // PortForwarder.remove(port);
            PortForwarder.add(port, limelightPath, port);
        }

        this.name = name;

        Rotation3d rotation = offset.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(name, offset.getX(), offset.getY(), offset.getZ(), rotation.getX(), rotation.getY(), rotation.getZ());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        for (int i = latency.length - 1; i >= 0; i--) {
            latency[i] = i == 0 ? LimelightHelpers.getLatency_Pipeline(name) : latency[i - 1];
        }

        inputs.isConnected = Arrays.stream(latency).distinct().count() == 1; // If the last n entries have the same latency

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        inputs.pose = estimate == null ? new Pose2d() : estimate.pose;
        Double[] hw = LimelightHelpers.getLimelightNTTableEntry(name, "hw").getDoubleArray(new Double[] {0.0, 0.0, 0.0, 0.0});

        inputs.fps = hw[0];
        inputs.cpuTemp = hw[1];
        inputs.ramUsage = hw[2];
        inputs.temp = hw[3];
    }

    @Override
    public PoseEstimate getEstimatedPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }
}
