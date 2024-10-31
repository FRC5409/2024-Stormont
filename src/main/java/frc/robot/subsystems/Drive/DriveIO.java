package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveIO {

    @AutoLog
    public class ModuleIOInputs {
        public boolean driveMotorConnected = false;
        public boolean steerMotorConnected = false;
        public boolean encoderConnected = false;

        public double driveVolts = 0.0;
        public double driveCurrent = 0.0;
        public double driveVelocity = 0.0;
        public double driveTemp = 0.0;

        public double steerVolts = 0.0;
        public double steerCurrent = 0.0;
        public Rotation2d steerPosition = Rotation2d.fromDegrees(0.0);
        public double steerTemp = 0.0;

        public Rotation2d absEncoderPosition = Rotation2d.fromDegrees(0.0);
    }

    @AutoLog
    public class GyroIOInputs {
        public boolean isConnected = false;
        
        public Rotation2d yaw = Rotation2d.fromDegrees(0.0);
        public Rotation2d pitch = Rotation2d.fromDegrees(0.0);
        public Rotation2d roll = Rotation2d.fromDegrees(0.0);
    }

    public void updateInputs(GyroIOInputs gyroInputs, ModuleIOInputs... moduleInputs);
}
