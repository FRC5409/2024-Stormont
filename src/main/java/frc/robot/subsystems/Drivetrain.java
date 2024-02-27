package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // subsystem
    private final PhotonVision sys_photonvision;

    // shuffleboard
    private final Field2d m_field;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        this.setDriveMotorsNeutralMode(NeutralModeValue.Brake);
        this.setTurnMotorsNeutralMode(NeutralModeValue.Brake);

        this.configurePathPlanner();

        this.seedFieldRelative();

        if (Utils.isSimulation()) {
            startSimThread();
        }

        // subsystems
        sys_photonvision = new PhotonVision();

        // shuffleboard
        m_field = new Field2d();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                this::driveFromChassisSpeeds, // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> false, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public void driveFromChassisSpeeds(ChassisSpeeds speeds) {
        this.setControl(autoRequest.withSpeeds(speeds));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setDriveMotorsNeutralMode(NeutralModeValue mode) {
        for (SwerveModule module : this.Modules) {
            module.getDriveMotor().setNeutralMode(mode);
        }
    }

    public void setTurnMotorsNeutralMode(NeutralModeValue mode) {
        for (SwerveModule module : this.Modules) {
            module.getSteerMotor().setNeutralMode(mode);
        }
    }

    public void setAllMotorsNeutralMode(NeutralModeValue mode) {
        this.setDriveMotorsNeutralMode(mode);
        this.setTurnMotorsNeutralMode(mode);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Updates Field2d on shuffleboard
     */
    private void updateFieldMap() {
        m_field.setRobotPose(m_odometry.getEstimatedPosition());
        SmartDashboard.putData(m_field);
    }

    /**
     * Returns list with positions of all swerve modules on robot
     * 
     * @return Position of all swerve modules
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < this.ModuleCount; i++) {
            SwerveModule module = this.Modules[i];
            positions[i] = module.getPosition(true);
        }

        return positions;
    }

    private void updatePoseEstimator() {
        var photonData = sys_photonvision.getEstimatedGlobalPose(this.getState().Pose);
        if (photonData.isPresent()) {
            // update pose estimator using april tags
            try {
                this.addVisionMeasurement(photonData.get().estimatedPose.toPose2d(), photonData.get().timestampSeconds);
            } catch (Exception e) {
                System.out.println(e);
            }
        }
    }

    public void periodic() {
        updatePoseEstimator();
        updateFieldMap();
    }

}
