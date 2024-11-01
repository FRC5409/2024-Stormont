package frc.robot.subsystems.Deployment;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.kDeployment;

public class DeploymentIOSparkMax implements DeploymentIO {
    private final CANSparkMax deploymentMotor;
    private final SparkPIDController deploymentController;
    private final RelativeEncoder deploymentEncoder;

    public DeploymentIOSparkMax(int ID) {
        deploymentMotor = new CANSparkMax(ID, MotorType.kBrushless);
        deploymentController = deploymentMotor.getPIDController();
        deploymentEncoder = deploymentMotor.getEncoder();

        deploymentMotor.restoreFactoryDefaults();
        deploymentMotor.setSmartCurrentLimit(kDeployment.CURRENT_LIMIT);
        deploymentMotor.setIdleMode(IdleMode.kBrake);

        deploymentMotor.setInverted(true);

        deploymentController.setP(kDeployment.kRealGains.KP);
        deploymentController.setI(kDeployment.kRealGains.KI);
        deploymentController.setD(kDeployment.kRealGains.KD);
        deploymentController.setFF(kDeployment.kRealGains.KFF);

        deploymentEncoder.setPositionConversionFactor((2 * Math.PI * kDeployment.ELEVATOR_DRUM_RADIUS) / kDeployment.ELEVATOR_GEARING);

        deploymentMotor.burnFlash();

        deploymentEncoder.setPosition(0.0);
    }

    private boolean isConnected(CANSparkMax motor) {
        // TOOD: Validate
        return !(motor.getFault(FaultID.kCANRX) || motor.getFault(FaultID.kCANTX) || motor.getFault(FaultID.kMotorFault));
    }

    @Override
    public void updateInputs(DeploymentIOInputs inputs) {
        inputs.motorConnected = isConnected(deploymentMotor);
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
