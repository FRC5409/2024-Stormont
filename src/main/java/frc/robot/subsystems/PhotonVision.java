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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCameras;
import frc.robot.Constants.kPhotonVision;

public class PhotonVision extends SubsystemBase {
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera frontCamera;
    PhotonCamera backCamera;
    PhotonPoseEstimator poseEstimatorFront;
    PhotonPoseEstimator poseEstimatorBack;
    private static PhotonVision instance = null;

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
    }

    /**
     * Returns Optional containing positioning data retrieved through april tags. If
     * there are no april tags visible, an empty optional will be returned.
     *
     * @return Positioning data
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> poseEstimateFront =
                getPoseEstimatorUpdate(frontCamera, poseEstimatorFront);
        Optional<EstimatedRobotPose> poseEstimateBack =
                getPoseEstimatorUpdate(backCamera, poseEstimatorBack);
        Optional<EstimatedRobotPose> poseEstimateOut = Optional.empty();

        if (poseEstimateFront.isPresent() && poseEstimateBack.isPresent()) {
            // pick one with better ambiguity
            if (getMeasurementAmbiguity(poseEstimateFront.get().targetsUsed)
                    < getMeasurementAmbiguity(poseEstimateBack.get().targetsUsed)) {
                poseEstimateOut = poseEstimateFront;
            } else {
                poseEstimateOut = poseEstimateBack;
            }
        } else if (poseEstimateFront.isPresent()) {
            poseEstimateOut = poseEstimateFront;
        } else if (poseEstimateBack.isPresent()) {
            poseEstimateOut = poseEstimateBack;
        }

        return poseEstimateOut;
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

    public Pose2d getNearestTagPoseWithOffset(Drivetrain sys_drivetrain, double offset) {
        Pose2d currentPose = sys_drivetrain.getAutoRobotPose();
        List<AprilTag> aprilTags = aprilTagFieldLayout.getTags();
        AprilTag closestTag = aprilTagFieldLayout.getTags().get(0);
        double closestDistance = getPoseDistance(currentPose, closestTag.pose.toPose2d());

        // Determining closest tag
        for (AprilTag tag : aprilTags) {
            double distance = getPoseDistance(currentPose, tag.pose.toPose2d());

            if (distance < closestDistance) {
                closestDistance = distance;
                closestTag = tag;
                // System.out.println(closestTag.ID);
            }
        }
        // System.out.println("=========================");
        // System.out.println(closestTag.pose.getRotation().getAngle());

        // Calculating target pose
        double x =
                closestTag.pose.getX()
                        + offset * Math.cos(closestTag.pose.getRotation().getAngle());
        double y =
                closestTag.pose.getY()
                        + offset * Math.sin(closestTag.pose.getRotation().getAngle());
        return new Pose2d(x, y, new Rotation2d(2.086 + 3.141592653589793));
    }

    private double getPoseDistance(Pose2d pose1, Pose2d pose2) {
        return Math.sqrt(
                Math.pow(pose2.getX() - pose1.getX(), 2)
                        + Math.pow(pose2.getY() - pose1.getY(), 2));
    }

    public static PhotonVision getInstance() {
        if (instance == null) {
            instance = new PhotonVision();
        }
        return instance;
    }

    @Override
    public void periodic() {}
}
