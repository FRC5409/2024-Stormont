// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class kControllers {
    public static final int kPrimaryController = 0;
    public static final int kSecondaryController = 1;
  }

  public static final class kCANID {

  }

  public static final class kDrive {

    public static final double kMaxDriveVelocity = 4.56; // metres per second
    public static final double kMaxDriveAcceleration = 2;
    public static final double kMaxTurnAngularVelocity = 10; // rotation per second
    public static final double kMaxTurnAngularAcceleration = 2 * Math.toRadians(360); // rotation per second squared

    public static final double kTranslationDeadband = 0.125;
    public static final double kTargetHeadingDeadband = 0.3;
    public static final double kManualRotationDeadband = 0.2;

    public static final double kDriveRampRate = 0.6;

    public static final double kHeadingSnap = Math.toRadians(45);

    public static final double kHeadingP = 5;
    public static final double kHeadingI = 0;
    public static final double kHeadingD = 0;

    public static final class kAutoAlign {
      public static final double kTControllerP = 5;
      public static final double kTControllerI = 0.0;
      public static final double kTControllerD = 0.0;
      public static final double kTControllerTolerance = 0.0;

      public static final double kRotationTolerance = 0.1;

      public static final double kReachedPositionTimeout = 500; // ms

      public static final boolean kAutoAlignDebug = false;
    }

  }

  public static final class kPhotonVision {
    public static final String kFieldLayout = AprilTagFields.k2024Crescendo.m_resourceFile;
    public static final double kAmbiguityThreshold = 0.4;
    // public static final Transform3d kFrontCameraOffset = new Transform3d(new
    // Translation3d(10, 0.0, 0.26416), new Rotation3d(0,0,0));
  }

  public static final class kCameras {
    // Arducam 1
    public static final String kFrontCameraName = "Front Camera";
    public static final String kFrontCameraID = "Arducam_OV2311_USB_Camera";
    public static final String kFrontCameraURL = "http://photonvision.local:1182/stream.mjpg";
    public static final Transform3d kFrontCameraOffset = new Transform3d(new Translation3d(0.3556, 0.0, 0.26416),
        new Rotation3d(0, 0, 0));

    // Arducam 2
    public static final String kBackCameraName = "Back Camera";
    public static final String kBackCameraID = "Arducam_OV2311_USB_Camera";
    public static final String kBackCameraURL = "placeholder";
    public static final Transform3d kBackCameraOffset = new Transform3d(new Translation3d(0, 0.0, 0),
        new Rotation3d(0, 0, 0));
  }

  public static final class kWaypoints {
    public static final Pose2d kAmpZoneTest = new Pose2d(14.5, 5.37, new Rotation2d(0, -.5)); // Variable
                                                                                              // positioning name is
                                                                                              // releative to f2d
                                                                                              // map on Shuffleboard
    // public static final Pose2d kAmpZoneTest = new Pose2d(14.5, 6.87, new
    // Rotation2d(0, .5)); //Variable positioning name is releative to f2d map on
    // Shuffleboard
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class kDeployment {
    public static final int id_motor = 0;
    public static final int digitalInputPort = 0;
    public static final int voltage = 0;
    public static final int currentLimit = 40;
    public static final int port_irSwitch = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static class setpoints {
      public static final double amp_pos = 0;
      public static final double trap_pos = 0;
      public static final double the_end = 0;
    }
  }

  public static class kCartridge {
    public static final int id_motor = 0;
    public static final int scoreVoltage = 0;
    public static final int voltage = 0;
    public static final int currentLimit = 0;
  }

  public static class kClimber {
    public static final int id_main = 0;
    public static final int id_follower = 0;
    public static final int port_limitSwitch = 0;
    public static final int port_irSwitch = 0;
    public static final int voltage = 0;
    public static final int currentLimit = 40;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double irZeroDistance = 0;

    public static final double high = 0;
    public static final double low = 0;

  }
}
