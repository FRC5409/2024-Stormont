package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
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

    // Simulation
    private final Mechanism2d deploymentMechanism;
    private final MechanismRoot2d deploymentRoot;
    private final MechanismLigament2d deploymentLigament;
    
    private final ElevatorSim sim_elevator;
    private final PIDController sim_controller;

    private final ShuffleboardTab simulationTab;

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
        } else {
            sim_elevator = null;
            deploymentMechanism = null;
            deploymentRoot = null;
            deploymentLigament = null;
            simulationTab = null;
            sim_controller = null;
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (RobotBase.isSimulation()) {
            sim_elevator.setInputVoltage(MathUtil.clamp((sim_controller.calculate(deploymentEncoder.getPosition()) * RoboRioSim.getVInVoltage() + kDeployment.KFF), -12, 12));

            sim_elevator.update(0.02);
            deploymentEncoder.setPosition(sim_elevator.getPositionMeters());

            deploymentLigament.setLength(deploymentEncoder.getPosition());
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
