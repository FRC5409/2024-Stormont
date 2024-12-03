package frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.kController;
import frc.robot.Constants.kDrive;
import frc.robot.generated.TunerConstants;

public class Drive extends SwerveDrivetrain implements Subsystem, DriveIO {

    private final GyroIOInputsAutoLogged gyroIO;
    private final ModuleIOInputsAutoLogged[] modulesIO;

    private final Pigeon2 m_pigeon;
    private final StatusSignal<Angle> pigeonYaw;
    private final StatusSignal<Angle> pigeonPitch;
    private final StatusSignal<Angle> pigeonRoll;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final ApplyRobotSpeeds m_autoRequest = new ApplyRobotSpeeds();
    private final FieldCentric m_telopDrive = new FieldCentric();

    // Shuffleboard
    private final Field2d m_field;

    private static Drive m_instance = null;

    // Get subsystem
    public static Drive getInstance() {
        if (m_instance == null) m_instance = TunerConstants.DriveTrain;

        return m_instance;
    }

    private Drive(SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... moduleConstants) {
        super(driveConstants, moduleConstants);

        gyroIO = new GyroIOInputsAutoLogged();
        modulesIO = new ModuleIOInputsAutoLogged[moduleConstants.length];
        for (int i = 0; i < modulesIO.length; i++) 
            modulesIO[i] = new ModuleIOInputsAutoLogged();

        m_pigeon = getPigeon2();
        pigeonYaw = m_pigeon.getYaw();
        pigeonPitch = m_pigeon.getPitch();
        pigeonRoll = m_pigeon.getRoll();

        m_poseEstimator = new SwerveDrivePoseEstimator(getKinematics(), m_pigeon.getRotation2d(), getState().ModulePositions, new Pose2d());

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // TODO: FALLBACK
            config = null;
            e.printStackTrace();
        }

        AutoBuilder.configure(
            () -> this.getState().Pose,
            this::resetPose,
            () -> this.getState().Speeds,
            this::driveFromChassisSpeeds,
            new PPHolonomicDriveController(
                kDrive.kPID.TRANSLATION, 
                kDrive.kPID.ROTATION
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        m_field = new Field2d();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public void driveFromChassisSpeeds(ChassisSpeeds speeds) {
        setControl(m_autoRequest.withSpeeds(speeds));
    }

    public Command telopDrive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationalRate) {
        return applyRequest(() -> m_telopDrive
            .withVelocityX(xSpeed.getAsDouble())
            .withVelocityY(ySpeed.getAsDouble())
            .withRotationalRate(rotationalRate.getAsDouble())
            .withDeadband(kController.kJoystickDeadband * kDrive.MAX_CHASSIS_SPEED)
            .withRotationalDeadband(kController.kTriggerDeadband * kDrive.MAX_ROTATION_SPEED)
        );
    }

    public Field2d getFieldMap() {
        return m_field;
    }

    @Override
    public void updateInputs(GyroIOInputs gyroInputs, ModuleIOInputs... moduleInputs) {
        if (getModules().length != moduleInputs.length) throw new IllegalArgumentException("Module count and ModuleInputs do not match in length!");

        gyroInputs.isConnected = BaseStatusSignal.isAllGood(pigeonYaw, pigeonPitch, pigeonRoll);

        gyroInputs.yaw = Rotation2d.fromDegrees(pigeonYaw.getValueAsDouble());
        gyroInputs.pitch = Rotation2d.fromDegrees(pigeonPitch.getValueAsDouble());
        gyroInputs.roll = Rotation2d.fromDegrees(pigeonRoll.getValueAsDouble());

        for (int i = 0; i < getModules().length; i++) {
            SwerveModule module = getModule(i);
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

    @AutoLogOutput(key = "Drive/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return getState().Pose;
    }

    @AutoLogOutput(key = "Drive/CurrentModuleState")
    public SwerveModuleState[] getCurrentState() {
        return getState().ModuleStates;
    }

    @AutoLogOutput(key = "Drive/TargetModuleState")
    public SwerveModuleState[] getTargetState() {
        return getState().ModuleTargets;
    }

    public void updateRobot() {
        m_poseEstimator.update(m_pigeon.getRotation2d(), getState().ModulePositions);

        // m_poseEstimator.addVisionMeasurement(getEstimatedPose(), m_drivetrainId);
    }

    public void periodic() {
        updateRobot();

        m_field.setRobotPose(getEstimatedPose());

        updateInputs(gyroIO, modulesIO);
        
        Logger.processInputs("Drive/Gyro", gyroIO);

        for (int i = 0; i < modulesIO.length; i++)
            Logger.processInputs("Drive/Module[" + kDrive.MODULE_NAMES[i] + "]", modulesIO[i]);
    }
}
