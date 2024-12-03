package frc.robot.subsystems.Deployment;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.kDeployment;

public class DeploymentIOSparkMax implements DeploymentIO {
    private final SparkMax deploymentMotor;
    private final SparkClosedLoopController deploymentController;
    private final RelativeEncoder deploymentEncoder;

    public DeploymentIOSparkMax(int ID) {
        deploymentMotor = new SparkMax(ID, MotorType.kBrushless);
        deploymentController = deploymentMotor.getClosedLoopController();
        deploymentEncoder = deploymentMotor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(kDeployment.CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);

        config.closedLoop.pidf(
            kDeployment.kRealGains.KP,
            kDeployment.kRealGains.KI,
            kDeployment.kRealGains.KD,
            kDeployment.kRealGains.KFF
        );

        config.encoder.positionConversionFactor((2 * Math.PI * kDeployment.ELEVATOR_DRUM_RADIUS) / kDeployment.ELEVATOR_GEARING);

        deploymentMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deploymentEncoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(DeploymentIOInputs inputs) {
        Faults faults = deploymentMotor.getFaults();

        inputs.motorConnected = !(faults.can || faults.motorType);
        inputs.appliedVoltage = deploymentMotor.get() * RobotController.getBatteryVoltage();
        inputs.appliedCurrent = deploymentMotor.getOutputCurrent();
        inputs.motorTemp = deploymentMotor.getMotorTemperature();
        inputs.position = deploymentEncoder.getPosition();
    }

    @Override
    public void setPosition(double position) {
        deploymentController.setReference(position, ControlType.kPosition);
    }

    @Override
    public double getPosition() {
        return deploymentEncoder.getPosition();
    }

    @Override
    public void stop() {
        deploymentMotor.stopMotor();
    }
}
