package frc.robot.subsystems.Deployment;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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
        return Commands.runOnce(() -> io.setPosition(kDeployment.EXTENSION_SETPOINT), this);
    }

    public Command retract() {
        return Commands.runOnce(() -> io.setPosition(kDeployment.STOW_SETPOINT), this);
    }

    public Command stop() {
        return Commands.runOnce(io::stop, this);
    }

    @AutoLogOutput(key = "Deployment/Pose")
    public Pose3d getDeploymentPose() {
        return new Pose3d(0, 0, inputs.position, new Rotation3d()).rotateBy(new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-15), Units.degreesToRadians(0)));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);

        sb_publisher.accept(getDeploymentPose());
        
        Logger.processInputs("Deployment", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
