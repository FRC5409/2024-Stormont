// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final class kDrive {

        public static final double kDriveGearRatio = 6.75;
        public static final double kTurnGearRatio = 150.0 / 7.0;

        public static final double kMaxDriveVelocity = 4.56; // metres per second
        public static final double kMaxTurnAngularVelocity = 10; // rotation per second
        public static final double kMaxTurnAngularAcceleration = 2 * Math.toRadians(360); // rotation per second squared

        public static final double kTranslationDeadband = 0.125;
        public static final double kTargetHeadingDeadband = 0.3;
        public static final double kManualRotationDeadband = 0.2;

        public static final double kDriveRampRate = 0.6;

        public static final double kHeadingSnap = Math.toRadians(45);

        public static final double kHeadingP = 2;
        public static final double kHeadingI = 0;
        public static final double kHeadingD = 0;

    }

    public static final class IntakeConstants {
        public static final int MOTOR_ID = 0;
        public static final int IR_SENSOR_PORT = 0;

        public static final int MOTOR_CURRENT_LIMIT = 30;
        public static final double MOTOR_RAMP_RATE = 0.1;

        public static final double HIGH_VOLTAGE = 10;
        public static final double LOW_VOLTAGE = 4;
    }

    public static final class IndexerConstants {
        public static final int MOTOR_ID = 0; // temp value
        public static final int RAMP_RATE = 1; // temp value
        public static final int CURRENT_LIMIT = 30;

        public static final int IR_SENSOR_PORT = 0; // temp value

        public static final double HIGH_VOLTAGE = 10;
        public static final double LOW_VOLTAGE = 4;
    }
}
