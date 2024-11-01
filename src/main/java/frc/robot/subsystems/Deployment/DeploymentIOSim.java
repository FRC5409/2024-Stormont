package frc.robot.subsystems.Deployment;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.kDeployment;

public class DeploymentIOSim implements DeploymentIO {
    private final Mechanism2d deploymentMechanism;
    private final MechanismRoot2d deploymentRoot;
    private final MechanismLigament2d deploymentLigament;

    private final ElevatorSim sim_elevator;
    private final PIDController sim_controller;

    public DeploymentIOSim() {
        deploymentMechanism = new Mechanism2d(0.2, 0.8);
        deploymentRoot = deploymentMechanism.getRoot("Deployment", 0, 0);
        deploymentLigament = deploymentRoot.append(new MechanismLigament2d("Deployment", kDeployment.LENGTH, 15));
        
        sim_elevator = new ElevatorSim(
            DCMotor.getNEO(1),
            kDeployment.ELEVATOR_GEARING,
            kDeployment.ELEVATOR_MASS,
            kDeployment.ELEVATOR_DRUM_RADIUS,
            0.0,
            kDeployment.LENGTH,
            true,
            0.0
        );

        sim_controller = new PIDController(kDeployment.kSimulationGains.KP, kDeployment.kSimulationGains.KI, kDeployment.kSimulationGains.KD);
        sim_controller.setTolerance(kDeployment.TOLERANCE);
    }

    @Override
    public void setPosition(double position) {
        sim_controller.setSetpoint(position);
    }

    @Override
    public void updateInputs(DeploymentIOInputs inputs) {
        updateSim();

        inputs.motorConnected = true;
        inputs.position = sim_elevator.getPositionMeters();
        inputs.appliedVoltage = MathUtil.clamp(sim_controller.calculate(sim_elevator.getPositionMeters()) * RoboRioSim.getVInVoltage(), -12, 12);
        inputs.appliedCurrent = sim_elevator.getCurrentDrawAmps();
        inputs.motorTemp = 0.0;
    }

    public void updateSim() {
        sim_elevator.setInputVoltage(MathUtil.clamp(sim_controller.calculate(sim_elevator.getPositionMeters()) * RoboRioSim.getVInVoltage(), -12, 12));

        sim_elevator.update(0.02);
        deploymentLigament.setLength(sim_elevator.getPositionMeters());
    }

    @Override
    public double getPosition() {
        return sim_elevator.getPositionMeters();
    }

    @Override
    public void stop() {
        // sim_elevator.stop
    }
}
