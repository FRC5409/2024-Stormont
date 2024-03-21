// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

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
        public static final int TEST_CONTROLLER = 2;
    }

    public static final class kRobot {
        public static final boolean IS_BETA_ROBOT = false;
        public static final boolean IS_HOME_FIELD = false;
    }

    public static final class kCANID {
        public static final int INTAKE_MOTOR_ID = 20;

        public static final int INDEXER_MOTOR_ID = 22;

        public static final int CLIMBER_MAIN_ID = 24;
        public static final int CLIMBER_FOLLOWER_ID = 25;

        public static final int DEPLOYMENT_MOTOR_ID = 15;
        public static final int CARTRIDGE_MOTOR_ID = 16;
    }

    public static final class kDrive {

        public static final ShuffleboardTab DRIVE_SHUFFLEBOARD_TAB = Shuffleboard.getTab("Drive");

        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        public static final double MAX_DRIVE_VELOCIY = 4.56; // metres per second
        public static final double MAX_DRIVE_ACCELERATION = 2;
        public static final double MAX_TURN_ANGULAR_VELOCITY = 10 / 2;
        public static final double MAX_TURN_ANGULAR_ACCELERATION = 2 * Math.toRadians(360);

        public static final double TRANSLATION_DEADBAND = 0.125;
        public static final double TARGET_HEADING_DEADBAND = 0.3;
        public static final double MANUAL_ROTATION_DEADBAND = 0.2;

        public static final double DRIVE_RAMP_RATE = 0.6;

        public static final double HEADING_SNAP = Math.toRadians(45);

        public static final double HEADING_P = 3;
        public static final double HEADING_I = 0;
        public static final double HEADING_D = 0;
        public static final double HEADING_FF = 0.4;

        public static final class kAutoPathPlanner {
            public static final double TRANSLATION_P = 2.5;
            public static final double TRANSLATION_I = 0;
            public static final double TRANSLATION_D = 0;

            public static final double ROTATION_P = 5;
            public static final double ROTATION_I = 0;
            public static final double ROTATION_D = 0;
        }

        public static final class kAutoAlign {
            public static final class kPIDDrive {
                public static final double T_CONTROLLER_P = 4.6;
                public static final double T_CONTROLLER_I = 0;
                public static final double T_CONTROLLER_D = .5;
                public static final double T_CONTROLLER_FF = .55;
                public static final double T_CONTROLLER_TOLERANCE = 0.01;
            }

            public static final class kPIDDriveSlow {
                public static final double T_CONTROLLER_P = 3;
                public static final double T_CONTROLLER_I = 0;
                public static final double T_CONTROLLER_D = 0;
                public static final double T_CONTROLLER_FF = .55;
                public static final double T_CONTROLLER_TOLERANCE = 0.005;
            }

            public static final double R_CONTROLLER_P = 10;
            public static final double R_CONTROLLER_I = 0;
            public static final double R_CONTROLLER_D = 1.5;
            public static final double R_CONTROLLER_FF = .45;
            public static final double ROTATION_TOLERANCE = 0.015;

            public static final double REACHED_POSITION_TOLERANCE = 0.1;
            public static final double REACHED_POSITION_TIMEOUT = 400; // ms

            public static final boolean AUTO_ALIGN_DEBUG = false;

            public static final class kAprilTags {
                public static final Map<Integer, Double> TRAP_TAG_ROTATIONS =
                        Map.of(
                                11,
                                2.086 - 2.094395102393195,
                                12,
                                2.086,
                                13,
                                2.086 + 2.094395102393195,
                                14,
                                2.086 - 2.094395102393195,
                                15,
                                2.086,
                                16,
                                2.086 + 2.094395102393195);
            }
        }
    }

    public static class kDeployment {
        public static final int DIO_PORT = 0;
        public static final int VOLTAGE = 3;
        public static final int MANUAL_VOLTAGE = 3;
        public static final int CURRENT_LIMIT = 40;
        public static final int IR_SWITCH_PORT = 0;
        public static final double TOLERANCE = 2;

        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static class kSetpoints {
            public static final double AMP_POSITION = -35;
            public static final double TRAP_POSITION = -47; // highest
            public static final double HOME = -0.25;
            public static final double LOW = -3;
            public static final double HIGH = -30;
            public static final double AMP_TRIGGER = -14;
            public static final double SHOOTING_TRIGGER = -30;
        }
    }

    public static class kCartridge {
        public static final int VOLTAGE = 7;
        public static final int MANUAL_VOLTAGE = 4;
        public static final int CURRENT_LIMIT = 0;
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
        public static final Transform3d FRONT_CAMERA_OFFSET =
                new Transform3d(
                        new Translation3d(0.418846, 0, 0.2234184),
                        // new Rotation3d(Math.toRadians(90), Math.toRadians(28), 0));
                        new Rotation3d(0, Math.toRadians(-30), 0)); // 0.3302 m towards the front

        // Arducam 2
        public static final String BACK_CAMERA_NAME = "Back Camera";
        public static final String BACK_CAMERA_ID = "OV2311_BackCamera";
        public static final String BACK_CAMERA_URL = "placeholder";
        public static final Transform3d BACK_CAMERA_OFFSET =
                new Transform3d(
                        new Translation3d(-0.2941828, 0.1674622, 0.2117598),
                        new Rotation3d(
                                0,
                                Math.toRadians(-42),
                                Math.toRadians(180))); // 0.3556 m towards the
        // BACK and 0.1524 m
        // to the left
    }

    public static final class kWaypoints {
        public static final Pose2d AMP_ZONE_TEST = new Pose2d(14.5, 5.37, new Rotation2d(0, -.5));
        public static final Pose2d AMP_ZONE_BLUE =
                new Pose2d(1.90, 7.73, new Rotation2d(0, Math.toRadians(-90)));
        public static final Pose2d AMP_ZONE_RED =
                new Pose2d(14.7, 7.73, new Rotation2d(0, Math.toRadians(-90)));
        public static final Pose2d TRAP_ZONE_15 =
                new Pose2d(4.26, 4.95, new Rotation2d(0, Math.toRadians(270)));
        public static final double TRAP_OFFSET = 0.28;
        public static final double TRAP_DISTANT_OFFSET = .67; 
    }

    public static final class kIntake {
        public static final int CURRENT_LIMIT = 50;

        public static final double VOLTAGE = 7;
    }

    public static final class kIndexer {
        public static final int CURRENT_LIMIT = 30;

        public static final int IR_SENSOR_PORT = 0;

        public static final double VOLTAGE = 8;
    }

    public static class kClimber {
        public static final int LIMIT_SWITCH_PORT = 0;
        public static final int IR_SWITCH_PORT = 0;
        public static final int VOLTAGE = 5;
        public static final int CURRENT_LIMIT = 40;
        public static final double CONVERSION_FACTOR = 0.692;

        public static final double KP_0 = 1;
        public static final double KI_0 = 0;
        public static final double KD_0 = 1;

        public static final double KP_1 = 0.2;
        public static final double kI_1 = 0;
        public static final double kD_1 = 0;

        public static final int KFAST_SLOT = 0;
        public static final int KSLOW_SLOT = 1;

        public static final double IR_ZERO_DISTANCE = 0;

        public static final double LOW = 0;
        public static final double MIDDLE = -33;
        public static final double HIGH = -60;
        public static final double TRAP_TRIGGER_POS = -40;
    }
}
