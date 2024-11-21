package frc.robot.subsystems.Drive;

import java.io.File;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import frc.robot.Constants;
import frc.robot.Constants.kController;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kMode;
import frc.robot.Constants.kDrive.kPID;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;

// 5409: The Chargers
// http://github.com/FRC5409

public class Drive extends SwerveDrivetrain implements Subsystem, DriveIO {
    private static Drive instance = null;

    private final GyroIOInputsAutoLogged gyroIO;
    private final ModuleIOInputsAutoLogged[] modulesIO;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.FieldCentric m_telopDrive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle m_alignDrive = new SwerveRequest.FieldCentricFacingAngle();
    private final SwerveRequest.ApplyChassisSpeeds m_autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveDrivePoseEstimator m_poseEstimator = 
        new SwerveDrivePoseEstimator(
            m_kinematics,
            m_pigeon2.getRotation2d(),
            m_modulePositions,
            new Pose2d()
        );

    private final Pigeon2 m_pigeon;
    private final StatusSignal<Double> pigeonYaw;
    private final StatusSignal<Double> pigeonPitch;
    private final StatusSignal<Double> pigeonRoll;

    // private final Vision sys_vision;

    private final PIDController autoAlignController;

    public final Field2d fieldMap;

    public Drive(SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... moduleConstants) {
        super(driveConstants, moduleConstants);

        gyroIO = new GyroIOInputsAutoLogged();
        modulesIO = new ModuleIOInputsAutoLogged[ModuleCount];
        for (int i = 0; i < modulesIO.length; i++)
            modulesIO[i] = new ModuleIOInputsAutoLogged();

        m_pigeon = getPigeon2();
        pigeonYaw   = m_pigeon.getYaw();
        pigeonPitch = m_pigeon.getPitch();
        pigeonRoll  = m_pigeon.getRoll();

        double driveBaseRadius = 0;
		for (Translation2d moduleLocation : m_moduleLocations) {
			driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
		}

        // sys_vision = Vision.getInstance();

        m_alignDrive.HeadingController.setP(kDrive.kPID.ROTATION_P);
        m_alignDrive.HeadingController.setI(kDrive.kPID.ROTATION_I);
        m_alignDrive.HeadingController.setD(kDrive.kPID.ROTATION_D);
        m_alignDrive.HeadingController.setTolerance(Math.toRadians(0.1));

        autoAlignController = new PIDController(kDrive.kPID.ROTATION_P, kDrive.kPID.ROTATION_I, kDrive.kPID.ROTATION_D);

        AutoBuilder.configureHolonomic(
            this::getEstimatedPose, 
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

        if (Constants.getMode() == kMode.SIM) {
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

    public Command driveTo(Supplier<Pose2d> pose) {
        return this.applyRequest(() -> {
            Pose2d setpoint = pose.get();
            Pose2d robot = getEstimatedPose();
    
            double xSpeed = autoAlignController.calculate(robot.getX(), setpoint.getX());
            double ySpeed = autoAlignController.calculate(robot.getY(), setpoint.getY());
    
            double currentAngle = robot.getRotation().getDegrees();
            double targetAngle = setpoint.getRotation().getDegrees();
    
            double angleDifference = targetAngle - currentAngle;

            if (angleDifference < -180)
                angleDifference += 360;
            if (angleDifference > 180)
                angleDifference -= 360;

            if (DriverStation.getAlliance().isPresent())
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    xSpeed *= -1;
                    ySpeed *= -1;
                }
    
            return this.m_alignDrive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withTargetDirection(Rotation2d.fromDegrees(Math.round(currentAngle + angleDifference)));
        });
    }

    public Command autoAlignSpeed(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds, Supplier<Rotation2d> targetAngle) {
        return this.applyRequest(() -> this.m_alignDrive
            .withVelocityX(-ySpeeds.getAsDouble())
            .withVelocityY(-xSpeeds.getAsDouble())
            .withTargetDirection(targetAngle.get())
        );
    }

    public Command telopDrive(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds, DoubleSupplier rotationalSpeed) {
        return this.applyRequest(() -> this.m_telopDrive
            .withVelocityX(-ySpeeds.getAsDouble())
            .withVelocityY(-xSpeeds.getAsDouble())
            .withRotationalRate(rotationalSpeed.getAsDouble())
            .withDeadband(kController.kJoystickDeadband * kDrive.MAX_CHASSIS_SPEED)
            .withRotationalDeadband(kController.kTriggerDeadband)
        );
    }

    public Command pointTowards(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds, Supplier<Pose2d> trackingPose) {
        return this.applyRequest(() -> {
            Pose2d robotPose = this.getEstimatedPose();
            Pose2d track = trackingPose.get();

            Rotation2d angle = Rotation2d.fromRadians(
                Math.atan2(robotPose.getY() - track.getY(), robotPose.getX() - track.getX()) + (DriverStation.getAlliance().get() == Alliance.Red ? Math.PI : 0.0)
            );

            return this.m_alignDrive
                .withVelocityX(-ySpeeds.getAsDouble())
                .withVelocityY(-xSpeeds.getAsDouble())
                .withTargetDirection(angle);
        });
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
        this.seedFieldRelative(getEstimatedPose());
    }

    @AutoLogOutput(key = "Drive/OdometryPose")
    public Pose2d getOdometryPose() {
        return m_odometry.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Drive/EstimatedPose")
    public Pose2d getEstimatedPose() {
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

    public void updateRobotPose() {
        m_poseEstimator.update(m_pigeon2.getRotation2d(), m_modulePositions);

        // PoseEstimate poseEstimate = sys_vision.getEstimatedPose();
        // if (poseEstimate == null)
        //     return;
        
        // m_poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }

    @AutoLogOutput(key = "Drive/ModuleStates")
    public SwerveModuleState[] getModuleState() {
        return m_moduleStates;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateInputs(gyroIO, modulesIO);
        
        // sys_vision.update();

        updateRobotPose();
        getModuleState();

        fieldMap.setRobotPose(getEstimatedPose());

        Logger.processInputs("Drive/Gyro", gyroIO);
        
        for (int i = 0; i < modulesIO.length; i++)
            Logger.processInputs("Drive/Module[" + kDrive.MODULE_NAMES[i] + "]", modulesIO[i]); // TODO: Validate modules names
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

    @Override
    public void updateInputs(GyroIOInputs gyroInputs, ModuleIOInputs... moduleInputs) {
        if (ModuleCount != moduleInputs.length) throw new IllegalArgumentException("Module count and ModuleInputs do not match in length!");

        gyroInputs.isConnected = BaseStatusSignal.isAllGood(pigeonYaw, pigeonPitch, pigeonRoll);

        gyroInputs.yaw = Rotation2d.fromDegrees(pigeonYaw.getValueAsDouble());
        gyroInputs.pitch = Rotation2d.fromDegrees(pigeonPitch.getValueAsDouble());
        gyroInputs.roll = Rotation2d.fromDegrees(pigeonRoll.getValueAsDouble());

        for (int i = 0; i < ModuleCount; i++) {
            SwerveModule module = getModule(0);
            TalonFX drive = module.getDriveMotor();
            TalonFX steer = module.getSteerMotor();
            CANcoder encoder = module.getCANcoder();

            moduleInputs[i].driveMotorConnected = BaseStatusSignal.isAllGood(drive.getVelocity(), drive.getStatorCurrent()); 
            moduleInputs[i].steerMotorConnected = BaseStatusSignal.isAllGood(steer.getPosition(), steer.getStatorCurrent()); // TODO: Validate steer position
            moduleInputs[i].encoderConnected = BaseStatusSignal.isAllGood(encoder.getAbsolutePosition());

            moduleInputs[i].driveVolts = drive.get() * RobotController.getBatteryVoltage();
            moduleInputs[i].steerVolts = steer.get() * RobotController.getBatteryVoltage();

            moduleInputs[i].driveCurrent = drive.getStatorCurrent().getValueAsDouble();
            moduleInputs[i].steerCurrent = steer.getStatorCurrent().getValueAsDouble();

            moduleInputs[i].driveVelocity = drive.getVelocity().getValueAsDouble();
            moduleInputs[i].steerPosition = Rotation2d.fromRotations(steer.getPosition().getValueAsDouble());

            moduleInputs[i].driveTemp = drive.getDeviceTemp().getValueAsDouble();
            moduleInputs[i].steerTemp = steer.getDeviceTemp().getValueAsDouble();

            moduleInputs[i].absEncoderPosition = Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
        }
    }

}