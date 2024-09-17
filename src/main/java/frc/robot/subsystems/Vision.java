package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

    private static Vision instance = null;

    private boolean DEBUG = true;

    private ShuffleboardTab sb_vision;

    private Field2d megaTag1Pose;
    private Field2d megaTag2Pose;

    private Vision() {
        Translation3d pos = kVision.limelightPoseOffset.getTranslation();
        Rotation3d rotation = kVision.limelightPoseOffset.getRotation();
        
        LimelightHelpers.setCameraPose_RobotSpace(
            kVision.name,
            pos.getX(),
            pos.getY(),
            pos.getZ(),
            rotation.getX(),
            rotation.getY(),
            rotation.getZ()
        );

        // Port Fowarding for limelight so you should be able to connect over USB
        for (int port = 5800; port < 5810; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

        if (DEBUG) {
            sb_vision = Shuffleboard.getTab("Vision");

            megaTag1Pose = new Field2d();
            megaTag2Pose = new Field2d();

            sb_vision.add("Megatag1", megaTag1Pose).withSize(5, 3).withPosition(0, 0);
            sb_vision.add("Megatag2", megaTag2Pose).withSize(5, 3).withPosition(5, 0);
        }
    }

    // Get subsystem
    public static Vision getInstance() {
        if (instance == null) instance = new Vision();

        return instance;
    }

    public PoseEstimate getVisionMeasurement() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kVision.name);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (DEBUG) {
            Pose2d mt1 = LimelightHelpers.getBotPose2d_wpiBlue(kVision.name);
            if (mt1 != null)
                megaTag1Pose.setRobotPose(mt1);

            PoseEstimate mt2 = getVisionMeasurement();
            if (mt2 != null)
                megaTag2Pose.setRobotPose(mt2.pose);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
