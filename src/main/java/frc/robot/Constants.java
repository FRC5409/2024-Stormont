// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        REPLAY,
        SIM
    }

    public static final kMode REAL_MODE = kMode.REAL;
    public static final kMode SIM_MODE = kMode.SIM;

    // PID variables
    public static class kPIDConstants {
        public static final double kPSpark = 0.0;
        public static final double kISpark = 0.0;
        public static final double kDSpark = 0.0;

        public static final double kPTalon = 0.0;
        public static final double kITalon = 0.0;
        public static final double kDTalon = 0.0;
    }

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
        public static final int kSecondaryController = 1;

        public static final double kJoystickDeadband = 0.05;
        public static final double kTriggerDeadband = 0.05;
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
}
