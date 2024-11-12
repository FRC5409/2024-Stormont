// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

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

    public static enum kMode {
        REAL,
        DEMO,
        REPLAY,
        SIM
    }

    public static final kMode REAL_MODE = kMode.REAL;
    public static final kMode SIM_MODE = kMode.SIM;

    /**
     * Gets the mode of the robot
     * WARNING: If the robot is booting and REAL_MODE was not set correctly for a comp match
     * it will return the set REAL_MODE until it connects to a FMS1
     * @return the given robot mode
     */
    public static kMode getMode() {
        if (RobotBase.isReal())
            return DriverStation.isFMSAttached() ? kMode.REAL : REAL_MODE;
        else 
            return SIM_MODE;
    }

    public static class kController {
        public static final int kDriverControllerPort = 0;

        public static final double kJoystickDeadband = 0.05;
        public static final double kTriggerDeadband = 0.05;
    }

    public static class kVision {
        public static final Transform3d LIMELIGHT_OFFSET = 
            new Transform3d(
		    	new Translation3d(0.1003965253, 0, 0.6490857384),
		    	new Rotation3d(0, Math.toRadians(145), 0)
            );
    }

    public static class kDrive {
        public static final String[] MODULE_NAMES = new String[] {
            "Front Left",
            "Front Right",
            "Back Left",
            "Back Right"
        };

        public static final int CURRENT_LIMIT = 100;

        public static final double MAX_CHASSIS_SPEED = 4.56;
        public static final double MAX_CHASSIS_ACCELERATION = 3.0;
        public static final double MAX_ROTATION_SPEED = 10.0;
        public static final double MAX_ROTATION_ACCELERATION = Math.toRadians(720.0);

        public static class kPID {
            public static final double TRANSLATION_P = 5.0;
            public static final double TRANSLATION_I = 0.0;
            public static final double TRANSLATION_D = 0.0;

            public static final double ROTATION_P = 7.0;
            public static final double ROTATION_I = 0.0;
            public static final double ROTATION_D = 0.0;
        }
    }

    public static final class kDeployment {
        public static final int DEPLOYMENT_ID = 15;
        public static final int CURRENT_LIMIT = 40;

        public static final class kRealGains {
            public static final double KP = 3.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double KFF = 0.0;
        }
        public static final class kSimulationGains {
            public static final double KP = 7.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double KFF = 0.0;
        }

        public static final double LENGTH = Units.inchesToMeters(17.55);

        public static final double ELEVATOR_ANGLE = 105;

        public static final double TOLERANCE = 0.01;

        public static final double EXTENSION_SETPOINT = 0.35;
        public static final double STOW_SETPOINT = 0.01;

        public static final double ELEVATOR_GEARING = 15.0/1.0;
        public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(0.944);
        public static final double ELEVATOR_MASS = Units.lbsToKilograms(11.0);
    }
}
