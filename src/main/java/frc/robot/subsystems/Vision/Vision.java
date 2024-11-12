package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs;

    private static Vision instance = null;

    private Vision(VisionIO io) {
        this.io = io;
        inputs = new VisionIOInputsAutoLogged();
    }

    public static Vision createInstance(VisionIO io) {
        if (instance != null) throw new RuntimeException("Limelight has already been created");

        return instance = new Vision(io);
    }

    // Get subsystem
    public static Vision getInstance() {
        if (instance == null) throw new RuntimeException("Limelight hasn't been created yet");

        return instance;
    }

    public PoseEstimate getEstimatedPose() {
        return io.getEstimatedPose();
    }

    /**
     * Should be called in the subsystem that uses vision before using any methods
     */
    public void update() {
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.processInputs("Limelight/" + io.getName(), inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}
