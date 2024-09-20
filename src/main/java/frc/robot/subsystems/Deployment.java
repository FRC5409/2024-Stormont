package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDeployment;

public class Deployment extends SubsystemBase {

    private static Deployment instance = null;

    private final CANSparkMax deploymentMotor;
    private final SparkPIDController deploymentController;
    private final RelativeEncoder deploymentEncoder;

    private double setpoint = kDeployment.MIN_HEIGHT;

    private double m_shooterAngle;

    // Simulation
    private final Mechanism2d deploymentMechanism;
    private final MechanismRoot2d deploymentRoot;
    private final MechanismLigament2d deploymentLigament;
    
    private final ElevatorSim sim_elevator;
    private final PIDController sim_controller;

    private final ShuffleboardTab simulationTab;
    private final StructPublisher<Pose3d> publisher;

    private Deployment() {
        deploymentMotor = new CANSparkMax(kDeployment.DEPLOYMENT_ID, MotorType.kBrushless);
        deploymentMotor.setSmartCurrentLimit(kDeployment.CURRENT_LIMIT);

        deploymentController = deploymentMotor.getPIDController();
        // Set PID coefficients (these values can be tuned)
        deploymentController.setP(kDeployment.KP);
        deploymentController.setI(kDeployment.KI);
        deploymentController.setD(kDeployment.KD);
        deploymentController.setFF(kDeployment.KFF);

        deploymentEncoder = deploymentMotor.getEncoder();
        deploymentEncoder.setPositionConversionFactor((2 * Math.PI * kDeployment.ELEVATOR_DRUM_RADIUS) / kDeployment.ELEVATOR_GEARING);

        m_shooterAngle = 0.0;

        resetEncoder();

        if (RobotBase.isSimulation()) {
            String name = "Deployment";

            deploymentMechanism = new Mechanism2d(0.20, 0.80);
            deploymentRoot = deploymentMechanism.getRoot(name, -0.1, 0.1);
            deploymentLigament = deploymentRoot.append(new MechanismLigament2d(name, kDeployment.MIN_HEIGHT, 105));

            sim_elevator = new ElevatorSim(
                DCMotor.getNEO(1),
                kDeployment.ELEVATOR_GEARING,
                kDeployment.ELEVATOR_MASS,
                kDeployment.ELEVATOR_DRUM_RADIUS,
                kDeployment.MIN_HEIGHT,
                kDeployment.MAX_HEIGHT,
                true,
                kDeployment.MIN_HEIGHT
            );

            sim_controller = new PIDController(kDeployment.KP, kDeployment.KI, kDeployment.KD);
            sim_controller.setTolerance(kDeployment.TOLERANCE);
            
            simulationTab = Shuffleboard.getTab("Simulation");
            simulationTab.add(name, deploymentMechanism);
            publisher = NetworkTableInstance.getDefault()
                .getStructTopic("Pose: " + name, Pose3d.struct).publish();
        } else {
            sim_elevator = null;
            deploymentMechanism = null;
            deploymentRoot = null;
            deploymentLigament = null;
            simulationTab = null;
            sim_controller = null;
            publisher = null;
        }

    }

    // Get subsystem
    public static Deployment getInstance() {
        if (instance == null) instance = new Deployment();

        return instance;
    }

    public void resetEncoder() {
        deploymentEncoder.setPosition(kDeployment.MIN_HEIGHT);
    }

    /**
     * Extends the deployment to a specified height
     * @param height in meters
     */
    public void extendTo(double height) {
        setpoint = height;

        deploymentController.setReference(height, ControlType.kPosition);

        if (RobotBase.isSimulation())
            sim_controller.setSetpoint(height);
    }

    public boolean atSetpoint() {
        if (RobotBase.isReal()) {
            return Math.abs(deploymentEncoder.getPosition() - setpoint) <= kDeployment.TOLERANCE;
        } else {
            return sim_controller.atSetpoint();
        }
    }

    public double getShooterAngle() {
        return m_shooterAngle;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (RobotBase.isSimulation()) {
            sim_elevator.setInputVoltage(MathUtil.clamp((sim_controller.calculate(deploymentEncoder.getPosition()) * RoboRioSim.getVInVoltage() + kDeployment.KFF), -12, 12));

            sim_elevator.update(0.02);
            deploymentEncoder.setPosition(sim_elevator.getPositionMeters());

            deploymentLigament.setLength(deploymentEncoder.getPosition());

            double length = deploymentLigament.getLength();

            publisher.accept(new Pose3d(-(length - kDeployment.MIN_HEIGHT) * Math.sin(Units.degreesToRadians(15)), 0, length * Math.cos(Units.degreesToRadians(15)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))));
        }

        Pose2d robotPose = Drive.getInstance().getRobotPose().plus(new Transform2d(new Translation2d(-0.26, 0), new Rotation2d()));

        Translation3d goal;
        if (DriverStation.getAlliance().isEmpty())
            goal = kDeployment.blueSpeaker;
        else
            goal = DriverStation.getAlliance().get() == Alliance.Blue ? kDeployment.blueSpeaker : kDeployment.redSpeaker;

        double z = goal.getZ() - 0.35;
        double d = Math.hypot(Math.abs(robotPose.getX() - goal.getX()), Math.abs(robotPose.getY() - goal.getY()));

        m_shooterAngle = Math.atan(z / d);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
