package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.AutoCreator.CustomAutoBuilder;
import frc.robot.Constants.kController;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kDrive.kPID;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

// 5409: The Chargers
// http://github.com/FRC5409

public class Drive extends SwerveDrivetrain implements Subsystem {
    private static Drive instance = null;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.FieldCentric m_telopDrive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle m_alignDrive = new SwerveRequest.FieldCentricFacingAngle();
    private final SwerveRequest.ApplyChassisSpeeds m_autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
			m_pigeon2.getRotation2d(), m_modulePositions, new Pose2d(),
			VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // TODO validate STDEVs
			VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1)));

    private final Vision sys_vision = Vision.getInstance();

    public final Field2d fieldMap;

    public Drive(SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... moduleConstants) {
        super(driveConstants, moduleConstants);

        double driveBaseRadius = 0;
		for (Translation2d moduleLocation : m_moduleLocations) {
			driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
		}

        CustomAutoBuilder.configureHolonomic(
            this::getRobotPose, 
            this::resetOdometry, 
            this::getChassisSpeeds, 
            this::driveFromChassisSpeeds,
            new HolonomicPathFollowerConfig(
                    new PIDConstants(kPID.TRANSLATION_P, kPID.TRANSLATION_I, kPID.TRANSLATION_D),
                    new PIDConstants(kPID.ROTATION_P, kPID.ROTATION_I, kPID.ROTATION_D),
                    kDrive.MAX_CHASSIS_SPEED,
                    driveBaseRadius,
                    new ReplanningConfig()
            ), 
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();

                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        updateFieldRelative();

        if (Utils.isSimulation()) {
            startSimThread();
        }

        fieldMap = new Field2d();
    }

    // Get subsystem
    public static Drive getInstance() {
        if (instance == null) instance = TunerConstants.DriveTrain;

        return instance;
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

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command autoAlignSpeed(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds, Supplier<Rotation2d> targetAngle) {
        return this.applyRequest(() -> this.m_alignDrive
            .withVelocityX(xSpeeds.getAsDouble())
            .withVelocityY(ySpeeds.getAsDouble())
            .withTargetDirection(targetAngle.get())
        );
    }

    public Command telopDrive(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds, DoubleSupplier rotationalSpeed) {
        return this.applyRequest(() -> this.m_telopDrive
            .withVelocityX(xSpeeds.getAsDouble())
            .withVelocityY(ySpeeds.getAsDouble())
            .withRotationalRate(rotationalSpeed.getAsDouble())
            .withDeadband(kController.kJoystickDeadband)
            .withRotationalDeadband(kController.kTriggerDeadband)
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(m_moduleStates);
    }

    public void driveFromChassisSpeeds(ChassisSpeeds speeds) {
        setControl(m_autoRequest.withSpeeds(speeds));
    }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(pose.getRotation(), m_modulePositions, pose);
        m_pigeon2.setYaw(pose.getRotation().getDegrees());
    }

    public void updateFieldRelative() {
        this.seedFieldRelative(getRobotPose());
    }

    public void updateRobotPose() {
        m_poseEstimator.update(m_pigeon2.getRotation2d(), m_modulePositions);

        PoseEstimate poseEstimate = sys_vision.getVisionMeasurement();
        if (poseEstimate == null)
            return;
        
        m_poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }

    public void addRotation(Rotation2d adder) {
        m_poseEstimator.resetPosition(getRobotPose().getRotation().plus(adder), m_modulePositions, getRobotPose());
    }

    public Pose2d getRobotPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Command[] getPathplannerCommands() {
        ArrayList<Command> commands = new ArrayList<Command>();

        File[] autos = new File(Filesystem.getDeployDirectory(), "pathplanner/autos/").listFiles();

        for (File auto : autos) {
            String autoName = auto.getName();

            autoName = autoName.substring(0, autoName.length() - 5); // Remove .auto

            commands.add(AutoBuilder.buildAuto(autoName).withName(autoName));
        }

        Command[] arr = new Command[commands.size()];
        arr = commands.toArray(arr);

        return arr;
    }

    public void putTrajectory(Supplier<Trajectory> trajSupplier) {
        fieldMap.getObject("Auto").setTrajectory(trajSupplier.get());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateRobotPose();
        fieldMap.setRobotPose(getRobotPose()); 
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}