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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
        public static final int PRIMARY_CONTROLLER = 0;
        public static final int SECONDARY_CONTROLLER = 1;
    }

    public static final class kRobot {
        public static final boolean IS_BETA_ROBOT = false;
    }

    public static final class kCANID {
        public static final int INTAKE_MOTOR_ID = 20;

        public static final int INDEXER_MOTOR_ID = 22;

        public static final int CLIMBER_MAIN_ID = 24;
        public static final int CLIMBER_FOLLOWER_ID = 25;
    }

    public static final class kDrive {

        public static final ShuffleboardTab DRIVE_SHUFFLEBOARD_TAB = Shuffleboard.getTab("Drive");

        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        public static final double MAX_DRIVE_VELOCIY = 4.56; // metres per second
        public static final double MAX_DRIVE_ACCELERATION = 2;
        public static final double MAX_TURN_ANGULAR_VELOCITY = 10;
        public static final double MAX_TURN_ANGULAR_ACCELERATION = 2 * Math.toRadians(360);

        public static final double TRANSLATION_DEADBAND = 0.125;
        public static final double TARGET_HEADING_DEADBAND = 0.3;
        public static final double MANUAL_ROTATION_DEADBAND = 0.2;

        public static final double DRIVE_RAMP_RATE = 0.6;

        public static final double HEADING_SNAP = Math.toRadians(45);

        public static final double HEADING_P = 5;
        public static final double HEADING_I = 0;
        public static final double HEADING_D = 0;

        public static final class kAutoPathPlanner {
            public static final double TRANSLATION_P = 3;
            public static final double TRANSLATION_I = 0;
            public static final double TRANSLATION_D = 0;

            public static final double ROTATION_P = 5;
            public static final double ROTATION_I = 0;
            public static final double ROTATION_D = 0;
        }

        public static final class kAutoAlign {
            public static final double T_CONTROLLER_P = 5;
            public static final double T_CONTROLLER_I = 0.0;
            public static final double T_CONTROLLER_D = 0.0;
            public static final double T_CONTROLLER_TOLERANCE = 0.0;

            public static final double ROTATION_TOLERANCE = 0.1;

            public static final double REACHED_POSITION_TIMEOUT = 500; // ms

            public static final boolean AUTO_ALIGN_DEBUG = false;
        }
    }

    public static final class kPhotonVision {
        public static final String FIELD_LAYOUT = AprilTagFields.k2024Crescendo.m_resourceFile;
        public static final double AMBIGUITY_THRESHOLD = 0.4;
        // public static final Transform3d kFrontCameraOffset = new Transform3d(new
        // Translation3d(10, 0.0, 0.26416), new Rotation3d(0,0,0));
    }

    public static final class kCameras {
        // OrangePIs
        public static final String FRONT_OPI = "photonvisionfront";
        public static final String BACK_OPI = "photonvisionback";

        // Arducam 1
        public static final String FRONT_CAMERA_NAME = "Front Camera";
        public static final String FRONT_CAMERA_ID = "OV2311_FrontCamera";
        public static final String FRONT_CAMERA_URL = "http://photonvision.local:1182/stream.mjpg";
        public static final Transform3d FRONT_CAMERA_OFFSET = new Transform3d(
                new Translation3d(0.418846, 0, 0.2234184),
                // new Rotation3d(Math.toRadians(90), Math.toRadians(28), 0));
                new Rotation3d(0, Math.toRadians(-30), 0)); // 0.3302 m towards the front

        // Arducam 2
        public static final String BACK_CAMERA_NAME = "Back Camera";
        public static final String BACK_CAMERA_ID = "OV2311_BackCamera";
        public static final String BACK_CAMERA_URL = "placeholder";
        public static final Transform3d BACK_CAMERA_OFFSET = new Transform3d(
                new Translation3d(-0.2941828, 0.1674622, 0.2117598),
                new Rotation3d(0, Math.toRadians(-57), Math.toRadians(180))); // 0.3556 m towards the
                                                                              // BACK and 0.1524 m
        // to the left
    }

    public static final class kWaypoints {
        public static final Pose2d AMP_ZONE_TEST = new Pose2d(14.5, 5.37, new Rotation2d(0, -.5)); // Variable
                                                                                                   // positioning name
                                                                                                   // is
                                                                                                   // releative to f2d
                                                                                                   // map on
                                                                                                   // Shuffleboard
        // public static final Pose2d kAmpZoneTest = new Pose2d(14.5, 6.87, new
        // Rotation2d(0, .5)); //Variable positioning name is releative to f2d map on
        // Shuffleboard
    }

    public static final class kIntake {
        public static final int CURRENT_LIMIT = 50;

        public static final double KP = 0.0003; // tuning in progress
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KFF = 0;

        public static final int RPM = 800; // tuning in progress
    }

    public static final class kIndexer {
        public static final int CURRENT_LIMIT = 30;

        public static final int IR_SENSOR_PORT = 0; // temporary

        public static final double VOLTAGE = 10;
    }

    public static class kClimber {
        public static final int LIMIT_SWITCH_PORT = 0;
        public static final int IR_SWITCH_PORT = 0;
        public static final int VOLTAGE = 3;
        public static final int CURRENT_LIMIT = 40;
        public static final double CONVERSION_FACTOR = 0.692;

        public static final double KP = 1;
        public static final double KI = 0;
        public static final double KD = 1;

        public static final double IR_ZERO_DISTANCE = 0;

        public static final double LOW = -1;
        public static final double MIDDLE = -33;
        public static final double HIGH = -60;

        // deployment elevator 23
        // index 22
        // intake 20
        // -63.79 (top when not inverted)

    }
}
