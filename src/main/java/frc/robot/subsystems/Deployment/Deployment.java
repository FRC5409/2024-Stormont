package frc.robot.subsystems.Deployment;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDeployment;

public class Deployment extends SubsystemBase {

    private final DeploymentIO io;
    private final DeploymentIOInputsAutoLogged inputs = new DeploymentIOInputsAutoLogged();

    private final ShuffleboardTab sb_deployment;
    private final StructPublisher<Pose3d> sb_publisher;

    private static Deployment instance = null;

    private Deployment(DeploymentIO io) {
        this.io = io;

        sb_deployment = Shuffleboard.getTab("Deployment");
        sb_publisher = NetworkTableInstance.getDefault().getStructTopic("Pose: Deployment", Pose3d.struct).publish();

        sb_deployment.addDouble("Position", io::getPosition);
    }

    public static Deployment createInstance(DeploymentIO io) {
        if (instance != null) throw new RuntimeException("Deployment has already been created");

        return instance = new Deployment(io);
    }

    // Get subsystem
    public static Deployment getInstance() {
        if (instance == null) throw new RuntimeException("Deployment hasn't been created yet");

        return instance;
    }

    public Command deploy() {
        return Commands.runOnce(() -> io.setPosition(kDeployment.LENGTH * 0.9), this);
    }

    public Command retract() {
        return Commands.runOnce(() -> io.setPosition(kDeployment.LENGTH * 0.05), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);

        sb_publisher.accept(inputs.deploymentPose);
        
        Logger.processInputs("Deployment", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
