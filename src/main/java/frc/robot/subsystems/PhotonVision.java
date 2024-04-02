// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCameras;
import frc.robot.Constants.kPhotonVision;
import frc.robot.Constants.kDrive.kAutoAlign;

public class PhotonVision extends SubsystemBase {
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera frontCamera, backCamera;
    private boolean enableFrontCamera, enableBackCamera; 
    PhotonPoseEstimator poseEstimatorFront;
    PhotonPoseEstimator poseEstimatorBack;
    private double lastResponseFront, lastResponseBack; 
    private static PhotonVision instance = null;

    private ShuffleboardTab sb_driveteamtab;

    public PhotonVision() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(kPhotonVision.FIELD_LAYOUT);
        } catch (Exception ignore) {
        }

        // Cameras
        // frontCamera = new PhotonCamera(kCameras.FRONT_CAMERA_ID);
        frontCamera = new PhotonCamera(kCameras.FRONT_CAMERA_ID);
        backCamera = new PhotonCamera(kCameras.BACK_CAMERA_ID);

        // Pose Estimator
        poseEstimatorFront =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        frontCamera,
                        kCameras.FRONT_CAMERA_OFFSET);
        poseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        poseEstimatorBack =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        backCamera,
                        kCameras.BACK_CAMERA_OFFSET);
        poseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        sb_driveteamtab = Shuffleboard.getTab("Drive team");
        sb_driveteamtab.addBoolean("FrontCamera", () -> frontCamera.isConnected()).withPosition(3, 0);
        sb_driveteamtab.addBoolean("BackCamera", () -> backCamera.isConnected()).withPosition(3, 1);

        this.enableFrontCamera = kPhotonVision.ENABLE_FRONT_CAMERA;
        this.enableBackCamera = kPhotonVision.ENABLE_BACK_CAMERA;
    }

    /**
     * Returns Optional containing positioning data retrieved through april tags. If
     * there are no april tags visible, an empty optional will be returned.
     *
     * @return Positioning data
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> poseEstimateFront = getPoseEstimatorUpdate(frontCamera, poseEstimatorFront);
        Optional<EstimatedRobotPose> poseEstimateBack = getPoseEstimatorUpdate(backCamera, poseEstimatorBack);
        Optional<EstimatedRobotPose> poseEstimateOut = Optional.empty();

        if (!enableFrontCamera) {
            System.out.println("FRONT CAMERA NOT ENABLED");
        }

        if (poseEstimateFront.isPresent() && poseEstimateBack.isPresent()) {
            // pick one with better ambiguity
            if (getMeasurementAmbiguity(poseEstimateFront.get().targetsUsed)
                    < getMeasurementAmbiguity(poseEstimateBack.get().targetsUsed) && enableFrontCamera) {
                poseEstimateOut = poseEstimateFront;
            } else if (enableBackCamera) {
                poseEstimateOut = poseEstimateBack;
            }
        } else if (poseEstimateFront.isPresent() && enableFrontCamera) {
            poseEstimateOut = poseEstimateFront;
        } else if (poseEstimateBack.isPresent() && enableBackCamera) {
            poseEstimateOut = poseEstimateBack;
        }
        return poseEstimateOut;
    }

    public void updateCameraStatus() {
        if (!frontCamera.isConnected()) {
            if ((System.currentTimeMillis() - lastResponseFront) > kPhotonVision.CAMERA_STATUS_TIMEOUT) {
                SmartDashboard.putBoolean("[PV] Front", false);
            }
        } else {
            lastResponseFront = System.currentTimeMillis();
            SmartDashboard.putBoolean("[PV] Front", true); //is this going to cause too much traffic?
        }

        if (!backCamera.isConnected()) {
            if ((System.currentTimeMillis() - lastResponseBack) > kPhotonVision.CAMERA_STATUS_TIMEOUT) {
                SmartDashboard.putBoolean("[PV] Back", false);
            }
        } else {
            lastResponseBack = System.currentTimeMillis();
            SmartDashboard.putBoolean("[PV] Back", true); //is this going to cause too much traffic?
        }
    }

    private Optional<EstimatedRobotPose> getPoseEstimatorUpdate(
            PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
        if (camera.isConnected()) {
            Optional<EstimatedRobotPose> photonData = poseEstimator.update();
            if (photonData.isPresent()) {
                return isWithinAmbiguityThreshold(
                                photonData.get().targetsUsed, kPhotonVision.AMBIGUITY_THRESHOLD)
                        ? photonData
                        : Optional.empty();
            }
        }
        return Optional.empty();
    }

    /**
     * Returns true if given targets are below specified threshold.
     *
     * @param targets   Targets to check for threshold
     * @param threshold Threshold for targets
     * @return Whether targets are above threshold
     */
    public boolean isWithinAmbiguityThreshold(List<PhotonTrackedTarget> targets, double threshold) {
        for (PhotonTrackedTarget target : targets) {
            if (target.getPoseAmbiguity() >= threshold) {
                return false;
            }
        }
        return true;
    }

    public double getMeasurementAmbiguity(List<PhotonTrackedTarget> targets) {
        double lowestAmbiguity = targets.get(0).getPoseAmbiguity();
        for (PhotonTrackedTarget target : targets) {
            if (target.getPoseAmbiguity() <= lowestAmbiguity) {
                lowestAmbiguity = target.getPoseAmbiguity();
            }
        }
        return lowestAmbiguity;
    }

    public Pose2d getNearestTagPoseWithOffset(
        Drivetrain sys_drivetrain, double offset, double targetRotation) {
        Pose2d currentPose = sys_drivetrain.getAutoRobotPose();
        List<AprilTag> aprilTags = aprilTagFieldLayout.getTags();
        AprilTag closestTag = aprilTagFieldLayout.getTags().get(11);
        double closestDistance = getPoseDistance(currentPose, closestTag.pose.toPose2d());

        // Determining closest tag
        for (AprilTag tag : aprilTags) {
            double distance = getPoseDistance(currentPose, tag.pose.toPose2d());

            if (distance < closestDistance
                    && kAutoAlign.kAprilTags.TRAP_TAG_ROTATIONS.containsKey(tag.ID)) {
                closestDistance = distance;
                closestTag = tag;
            }
        }

        double x =
                closestTag.pose.getX()
                        + offset * Math.cos(closestTag.pose.getRotation().getAngle());
        double y =
                closestTag.pose.getY()
                        + offset * Math.sin(closestTag.pose.getRotation().getAngle());
        return new Pose2d(x, y, new Rotation2d(targetRotation));
    }

    private double getPoseDistance(Pose2d pose1, Pose2d pose2) {
        return Math.abs(Math.sqrt(
                Math.pow(pose2.getX() - pose1.getX(), 2)
                        + Math.pow(pose2.getY() - pose1.getY(), 2)));
    }

    public static PhotonVision getInstance() {
        if (instance == null) {
            instance = new PhotonVision();
        }
        return instance;
    }

    public void setCameraEnableStatus(boolean isEnabled, String camera) {
        switch (camera) {
            case "Front":
                this.enableFrontCamera = isEnabled;
                break;
            case "Back":
                this.enableBackCamera = isEnabled;
                break;
        }
    }

    @Override
    public void periodic() {}
}

